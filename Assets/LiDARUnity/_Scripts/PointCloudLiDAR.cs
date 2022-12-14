// author: Gustavo Beltrao Braga
// description:
//   LiDAR Plugin for Unity.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.ARFoundation;
using UnityEngine.XR.ARSubsystems;
using Unity.Collections.LowLevel.Unsafe;
using System.Threading;
using System.IO;

public class PointCloudLiDAR : MonoBehaviour
{
    [SerializeField] AROcclusionManager occlusionManager;
    [SerializeField] ARCameraManager arCameraManager;
    [SerializeField] Material pointCloudMaterial;

    #region Variables
    [HideInInspector] public Texture2D cameraTexture, depthTextureFloat, depthTextureBGRA, depthConfidenceTextureR8, depthConfidenceTextureRGBA;
    [HideInInspector] public int frameCount = 0;
    [HideInInspector] public GameObject pointCLoudMaster, postProcessedChunks;
    [HideInInspector] public Camera mainCamera;
    Dictionary<Vector3, Chunk> pointCloudChunks;
    HashSet<Vector3> chunksThatAreFull;
    GameObject processedChunks, chunksThatNeedToBeDeleted;
    bool needToUpdateIntrinsics = true;
    float cx, cy, fx, fy;
    int totalPoints = 0, totalMeshes = 0, nMeshesToCreate = 0;
    int verticesPPostProcessedMesh = 25000;
    Vector3 camPosition, lastCamPosition;
    Quaternion camRotation, lastCamRotation;
    #endregion

    #region Voxel Downsampling Grid Variables
    List<Vector3> voxelGridVertices = new List<Vector3>();
    List<Vector3> voxelGridNormals = new List<Vector3>();
    List<Color> voxelGridColors = new List<Color>();
    int readyChunksCount = 0;
    #endregion

    #region Thread Variables
    List<MeshFilter> meshToRemove = new List<MeshFilter>();
    List<Vector3> verticesInThread = new List<Vector3>();
    List<Vector3> normalsInThread = new List<Vector3>();
    List<Color> colorsInThread = new List<Color>();
    int meshesSaved = 0;
    Thread thread;
    #endregion

    #region Scan Settings
    [HideInInspector] public int numberOfReadyChunksNeededToPostProcess = 8;
    [HideInInspector] public float minRotationThresholdDegree = .011f;
    [HideInInspector] public float minTranslationThreshold = .00011f;
    [HideInInspector] public int maxVerticesPerChunk = 8192 * 2;
    [HideInInspector] public int maxVerticesPerNewMesh = 8192;
    [HideInInspector] public float maxPointsPerFrame = .5f;
    [HideInInspector] public int maxMeshesPerChunk = 6;
    [HideInInspector] public int divideChunkInto = 4;
    [HideInInspector] public float chunkSize = .5f;
    [HideInInspector] public int frameWait = 30;
    [HideInInspector] public float near = .2f;
    [HideInInspector] public float far = 4;
    [HideInInspector] public bool threadedMeshOptimization = true;
    [HideInInspector] public bool useVoxelDownSampling = true;
    [HideInInspector] public bool isScanning = false;
    [HideInInspector] public bool useTresh = true;
    [HideInInspector] public bool fastNormalEstimation = false;
    [HideInInspector] public bool forceOrganizedPointCloud = true;
    [HideInInspector] public bool showNormals = false;
    #endregion

    #region Delegates
    public delegate void OnPointCloudImagesReady();
    public OnPointCloudImagesReady onPointCloudImagesReady;
    #endregion

    #region Structs
    struct PointCloudChunk
    {
        public List<Vector3> vertices;
        public List<Vector3> normals;
        public List<Color> colors;
        public Vector4[] sumOfVertices;
        public Vector3[] sumOfNormals;
        public Color[] sumOfColors;
    }

    struct Chunk
    {
        public int chunkCount;
        public int verticesCount;
        public Vector4[] sumOfVertices;
        public Vector3[] sumOfNormals;
        public Color[] sumOfColors;
    }
    #endregion
    
    #region Unity Methods
    void OnEnable() 
    { 
        arCameraManager.frameReceived += OnFrameReceived;
        SetupScan();
    }

    void OnDisable() 
    {
        if (threadedMeshOptimization)
            thread.Abort();
        arCameraManager.frameReceived -= OnFrameReceived; 
    }

    void OnFrameReceived(ARCameraFrameEventArgs eventArgs)
    {
        UpdateCameraImage();
        UpdateEnvironmentDepthImage();
        UpdateEnvironmentConfidenceImage();

        if (onPointCloudImagesReady != null)
            onPointCloudImagesReady();

        if (isScanning && frameCount >= frameWait) 
        {
            frameCount = 0;
            lastCamPosition = camPosition;
            lastCamRotation = camRotation;
            camPosition = arCameraManager.transform.position;
            camRotation = arCameraManager.transform.rotation;

            if (useTresh && ((camPosition - lastCamPosition).magnitude < minTranslationThreshold * frameWait || (camRotation.eulerAngles - lastCamRotation.eulerAngles).magnitude < minRotationThresholdDegree * frameWait))
                return;
            
            if (useVoxelDownSampling)
                ProcessPointCloudDataWithVoxelGrid();
            else
                ProcessPointCloudData();

            if (threadedMeshOptimization && !thread.IsAlive)
            {
                CreateMeshesInThread();
                thread.Start();
            }
        }

        frameCount ++;
    }

    void OnApplicationQuit()
    {
        if (thread != null) thread.Abort();
    }
    #endregion

    #region Image processing
    void UpdateCameraImage()
    {
       if (!arCameraManager.TryAcquireLatestCpuImage(out XRCpuImage image))
            return;

        using (image)
        {
            var format = TextureFormat.RGBA32;

            if (cameraTexture == null || cameraTexture.width != image.width || cameraTexture.height != image.height)
                cameraTexture = new Texture2D(image.width, image.height, format, false);

            UpdateImage(cameraTexture, image, format);
        }
    }

    void UpdateEnvironmentDepthImage()
    {
        if (!occlusionManager.TryAcquireEnvironmentDepthCpuImage(out XRCpuImage image))
            return;

        using (image)
        {
            if (depthTextureFloat == null || depthTextureFloat.width != image.width || depthTextureFloat.height != image.height)
                depthTextureFloat = new Texture2D(image.width, image.height, image.format.AsTextureFormat(), false);

            if (depthTextureBGRA == null || depthTextureBGRA.width != image.width || depthTextureBGRA.height != image.height)
                depthTextureBGRA = new Texture2D(image.width, image.height, TextureFormat.BGRA32, false);

            UpdateImage(depthTextureFloat, image, image.format.AsTextureFormat());
            FloatToGrayScale(depthTextureFloat, depthTextureBGRA);
        }
    }

    void UpdateEnvironmentConfidenceImage()
    {
        if (!occlusionManager.TryAcquireEnvironmentDepthConfidenceCpuImage(out XRCpuImage image))
            return;

        using (image)
        {
            if (depthConfidenceTextureR8 == null || depthConfidenceTextureR8.width != image.width || depthConfidenceTextureR8.height != image.height)
                depthConfidenceTextureR8 = new Texture2D(image.width, image.height, image.format.AsTextureFormat(), false);

            if (depthConfidenceTextureRGBA == null || depthConfidenceTextureRGBA.width != image.width || depthConfidenceTextureRGBA.height != image.height)
                depthConfidenceTextureRGBA = new Texture2D(image.width, image.height, TextureFormat.BGRA32, false);
                
            UpdateImage(depthConfidenceTextureR8, image, image.format.AsTextureFormat());
            R8ToConfidenceMap(depthConfidenceTextureR8, depthConfidenceTextureRGBA);
        }
    }

    unsafe void UpdateImage(Texture2D texture, XRCpuImage cpuImage, TextureFormat format)
    {
        var conversionParams = new XRCpuImage.ConversionParams(cpuImage, format, XRCpuImage.Transformation.MirrorY);
        var rawTextureData = texture.GetRawTextureData<byte>();

        Debug.Assert(rawTextureData.Length == cpuImage.GetConvertedDataSize(conversionParams.outputDimensions, conversionParams.outputFormat),
            "The Texture2D is not the same size as the converted data.");

        cpuImage.Convert(conversionParams, rawTextureData);
        texture.Apply();
    }

    void FloatToGrayScale(Texture2D txFloat, Texture2D txGray)
    {
        int length = txGray.width * txGray.height;
        Color[] depthPixels = txFloat.GetPixels();
        Color[] colorPixels = txGray.GetPixels();

        for (int index = 0; index < length; index++)
        {
            var value = (depthPixels[index].r - near) / (far - near);
            colorPixels[index] = new Color(value, value, value, 1.0f);
        }

        txGray.SetPixels(colorPixels);
        txGray.Apply();
    }

    void R8ToConfidenceMap(Texture2D txR8, Texture2D txRGBA)
    {
        Color32[] r8 = txR8.GetPixels32();
        Color32[] rgba = txRGBA.GetPixels32();

        for (int i = 0; i < r8.Length; i++)
            switch (r8[i].r)
            {
                case 0:
                    rgba[i] = new Color32(255, 0, 0, 255);
                    break;
                case 1:
                    rgba[i] = new Color32(0, 255, 0, 255);
                    break;
                case 2:
                    rgba[i] = new Color32(0, 0, 255, 255);
                    break;
            }

        txRGBA.SetPixels32(rgba);
        txRGBA.Apply();
    }
    #endregion

    #region Process Point Cloud
    void ProcessPointCloudDataWithVoxelGrid()
    {
        double lastInterval = Time.realtimeSinceStartup;
        Dictionary<Vector3, PointCloudChunk> pointCloudFrameChunk = new Dictionary<Vector3, PointCloudChunk>();
        int divideChunkIntoCube = divideChunkInto * divideChunkInto * divideChunkInto;

        int width_depth = depthTextureFloat.width;
        int height_depth = depthTextureFloat.height;
        int width_camera = cameraTexture.width;
        int height_camera = cameraTexture.height;

        if (needToUpdateIntrinsics)
        {
            XRCameraIntrinsics intrinsic;
            arCameraManager.TryGetIntrinsics(out intrinsic);

            float ratio = (float) width_depth / (float) width_camera;
            fx = intrinsic.focalLength.x * ratio;
            fy = intrinsic.focalLength.y * ratio;

            cx = intrinsic.principalPoint.x * ratio;
            cy = intrinsic.principalPoint.y * ratio;
            needToUpdateIntrinsics = false;
        }

        Color[] depthPixels = depthTextureFloat.GetPixels();
        Color32[] confidenceMap = depthConfidenceTextureR8.GetPixels32(); 
        Matrix4x4 trs = Matrix4x4.TRS(camPosition, camRotation, new Vector3(1, 1, 1));
        Vector3 halfChunkSize = new Vector3(chunkSize, chunkSize, chunkSize) / 2f;

        int index_dst;
        float depth, u, v;
        float width_height = width_depth * height_depth;
        
        int pointsPerLine = (int) (width_height * maxPointsPerFrame / height_depth);
        float jumpValue = (width_depth / pointsPerLine) - 1f;
        float jumpCounter = jumpValue;
        Vector3 vertice, normal;
        Color color;

        for(int depth_y = 0; depth_y < height_depth; depth_y ++)
        {
            index_dst = depth_y * width_depth;
            v = (float) depth_y / (height_depth) * height_camera;

            for(int depth_x = 0; depth_x < width_depth; depth_x ++)
            {
                if (jumpCounter <= 1f)
                {
                    jumpCounter += jumpValue;
                    depth = depthPixels[index_dst].r;
                    
                    if (depth > near && depth < far && confidenceMap[index_dst].r == 2)
                    {
                        vertice.z = depth;
                        vertice.x = - depth * (depth_x - cx) / fx;
                        vertice.y = - depth * (depth_y - cy) / fy;
                        vertice = trs.MultiplyPoint(vertice);

                        if (vertice == Vector3.zero)
                        {
                            index_dst ++;
                            continue;
                        }

                        Vector3 chunkPos = new Vector3(Mathf.Round(vertice.x / chunkSize), Mathf.Round(vertice.y / chunkSize), Mathf.Round(vertice.z / chunkSize));
                        if (chunksThatAreFull.Contains(chunkPos))
                        {
                            index_dst ++;
                            continue;
                        }

                        u = (float) depth_x / (width_depth);
                        color = cameraTexture.GetPixel((int) (u * width_camera), (int) v);
                        normal = (mainCamera.transform.position - vertice).normalized;

                        Vector4 v4 = new Vector4(vertice.x, vertice.y, vertice.z, 1);
                        Vector3 verticesInSubChunk = new Vector3(((vertice.x - chunkPos.x * chunkSize + chunkSize / 2) / chunkSize) * divideChunkInto,
                                                    ((vertice.y - chunkPos.y * chunkSize + chunkSize / 2) / chunkSize) * divideChunkInto,
                                                    ((vertice.z - chunkPos.z * chunkSize + chunkSize / 2) / chunkSize) * divideChunkInto);

                        int posInArray = ((int) (verticesInSubChunk.x) + (int) (verticesInSubChunk.y) * divideChunkInto + (int) (verticesInSubChunk.z) * divideChunkInto * divideChunkInto);
                        Debug.Log("posInArray: " + posInArray + " verticesInSubChunk: " + verticesInSubChunk + " vertice: " + vertice + " chunkPos: " + chunkPos);
                        if (pointCloudFrameChunk.ContainsKey(chunkPos))
                        {
                            if (pointCloudFrameChunk[chunkPos].vertices.Count > maxVerticesPerNewMesh)
                            {
                                index_dst ++;
                                continue;
                            }

                            pointCloudFrameChunk[chunkPos].vertices.Add(vertice);
                            pointCloudFrameChunk[chunkPos].sumOfVertices[posInArray] += v4;
                            pointCloudFrameChunk[chunkPos].sumOfNormals[posInArray] += normal;
                            pointCloudFrameChunk[chunkPos].sumOfColors[posInArray] += color;
                        }

                        else
                        {
                            PointCloudChunk pc = new PointCloudChunk();
                            pc.vertices = new List<Vector3>() { vertice };
                            pc.sumOfVertices = new Vector4[divideChunkIntoCube];
                            pc.sumOfNormals = new Vector3[divideChunkIntoCube];
                            pc.sumOfColors = new Color[divideChunkIntoCube];

                            pc.sumOfVertices[posInArray] = v4;
                            pc.sumOfNormals[posInArray] = normal;
                            pc.sumOfColors[posInArray] = color;

                            pointCloudFrameChunk.Add(chunkPos, pc);
                        }
                    }
                }

                else jumpCounter -= 1f;
                index_dst ++;
            }
        }

        double timeNow = Time.realtimeSinceStartup;
        Print("First step: " + (timeNow - lastInterval));
        lastInterval = Time.realtimeSinceStartup;
        
        foreach (Vector3 chunk in pointCloudFrameChunk.Keys)
        {
            if (pointCloudChunks.ContainsKey(chunk))
            {
                int verticesCount = pointCloudChunks[chunk].verticesCount + pointCloudFrameChunk[chunk].vertices.Count;
                int idx = pointCloudFrameChunk[chunk].vertices.Count;

                if (verticesCount > maxVerticesPerChunk) 
                    idx -= (verticesCount - maxVerticesPerChunk);

                if (pointCloudChunks[chunk].chunkCount <= maxMeshesPerChunk && idx >= 0)
                {
                    Chunk c = pointCloudChunks[chunk];
                    c.chunkCount ++;
                    c.verticesCount = verticesCount;
                    
                    for (int i = 0; i < divideChunkIntoCube; i ++)
                    {
                        c.sumOfVertices[i] += pointCloudFrameChunk[chunk].sumOfVertices[i];
                        c.sumOfNormals[i] += pointCloudFrameChunk[chunk].sumOfNormals[i];
                        c.sumOfColors[i] += pointCloudFrameChunk[chunk].sumOfColors[i];
                    }

                    pointCloudChunks[chunk] = c;
                    totalMeshes ++;
                    totalPoints += pointCloudFrameChunk[chunk].vertices.Count;
                }

                else
                {
                    readyChunksCount ++;
                    chunksThatAreFull.Add(chunk);

                    if (fastNormalEstimation)
                    {
                        List<Vector3> meanVertices = new List<Vector3>();
                        List<Vector3> meanNormals = new List<Vector3>();

                        for (int j = 0; j < pointCloudChunks[chunk].sumOfVertices.Length; j ++)
                        {
                            if (pointCloudChunks[chunk].sumOfVertices[j].w == 0) continue;

                            Vector3 meanVertex = pointCloudChunks[chunk].sumOfVertices[j] / pointCloudChunks[chunk].sumOfVertices[j].w;
                            meanVertices.Add(meanVertex);
                            meanNormals.Add(pointCloudChunks[chunk].sumOfNormals[j] / pointCloudChunks[chunk].sumOfVertices[j].w);

                            voxelGridVertices.Add(meanVertex);
                            voxelGridColors.Add(pointCloudChunks[chunk].sumOfColors[j] / pointCloudChunks[chunk].sumOfVertices[j].w);
                        }

                        for (int j = 0; j < meanVertices.Count; j ++)
                        {
                            Vector3 crossTangent = Vector3.zero;

                            for (int k = 0; k < meanVertices.Count; k ++)
                                if (k != j)
                                    crossTangent += Vector3.Cross(meanVertices[j] - meanVertices[k], meanVertices[(j + 1) % meanVertices.Count] - meanVertices[k]);

                            crossTangent.Normalize();
                            float sign = Vector3.Dot(crossTangent, meanNormals[j]) > 0 ? 1 : -1;
                            crossTangent *= sign;

                            if (showNormals)
                            {
                                GameObject myLine = new GameObject();
                                myLine.transform.parent = pointCLoudMaster.transform;
                                myLine.transform.position = meanVertices[j];
                                myLine.AddComponent<LineRenderer>();
                                LineRenderer lr = myLine.GetComponent<LineRenderer>();
                                lr.material = new Material(Shader.Find("Unlit/Color"));
                                lr.material.color = Color.red;
                                lr.SetWidth(0.005f, 0.005f);
                                lr.SetPosition(0, meanVertices[j]);
                                lr.SetPosition(1, meanVertices[j] + crossTangent / 60f);
                            }

                            voxelGridNormals.Add(crossTangent);
                        }
                    }

                    else 
                    {
                        Vector3 worldChunkPos = (chunk * chunkSize) - halfChunkSize;
                        for (int j = 0; j < pointCloudChunks[chunk].sumOfVertices.Length; j++)
                        {
                            if (pointCloudChunks[chunk].sumOfVertices[j].w == 0) continue;

                            if (forceOrganizedPointCloud)
                            {
                                float x = j % divideChunkInto;
                                float y = (j / divideChunkInto) % divideChunkInto;
                                float z = j / (divideChunkInto * divideChunkInto);

                                voxelGridVertices.Add(worldChunkPos + new Vector3(x, y, z) * chunkSize / divideChunkInto + halfChunkSize / divideChunkInto);
                            }
                            else
                                voxelGridVertices.Add(pointCloudChunks[chunk].sumOfVertices[j] / pointCloudChunks[chunk].sumOfVertices[j].w);

                            voxelGridNormals.Add(pointCloudChunks[chunk].sumOfNormals[j] / pointCloudChunks[chunk].sumOfVertices[j].w);
                            voxelGridColors.Add(pointCloudChunks[chunk].sumOfColors[j] / pointCloudChunks[chunk].sumOfVertices[j].w);
                        }
                    }

                    Chunk c = pointCloudChunks[chunk];
                    c.sumOfColors = null;
                    c.sumOfNormals = null;
                    c.sumOfVertices = null;
                    pointCloudChunks[chunk] = c;
                }
            }

            else
            {
                int idx = pointCloudFrameChunk[chunk].vertices.Count;
                if (idx > maxVerticesPerChunk) 
                    idx -= (idx - maxVerticesPerChunk);

                Chunk c = new Chunk();
                c.chunkCount = 1;
                c.verticesCount = idx;

                c.sumOfVertices = new Vector4[divideChunkIntoCube];
                c.sumOfNormals = new Vector3[divideChunkIntoCube];
                c.sumOfColors = new Color[divideChunkIntoCube];

                for (int i = 0; i < divideChunkIntoCube; i ++)
                {
                    c.sumOfVertices[i] = pointCloudFrameChunk[chunk].sumOfVertices[i];
                    c.sumOfNormals[i] = pointCloudFrameChunk[chunk].sumOfNormals[i];
                    c.sumOfColors[i] = pointCloudFrameChunk[chunk].sumOfColors[i];
                }

                pointCloudChunks.Add(chunk, c);
                totalMeshes ++;
                totalPoints += idx;
            }
        }

        timeNow = Time.realtimeSinceStartup;
        Print("Second step: " + (timeNow - lastInterval));

        if (readyChunksCount >= numberOfReadyChunksNeededToPostProcess)
        {
            int meshCount = (int) (voxelGridVertices.Count / verticesPPostProcessedMesh) + 1;
            CreateLoadOfMeshes(meshCount, voxelGridVertices, voxelGridColors, voxelGridNormals, processedChunks.transform);
            voxelGridVertices = new List<Vector3>();
            voxelGridColors = new List<Color>();
            voxelGridNormals = new List<Vector3>();
            readyChunksCount = 0;
        }
    }

    void ProcessPointCloudData()
    {
        double lastInterval = Time.realtimeSinceStartup;
        Dictionary<Vector3, PointCloudChunk> pointCloudFrameChunk = new Dictionary<Vector3, PointCloudChunk>();

        int width_depth = depthTextureFloat.width;
        int height_depth = depthTextureFloat.height;
        int width_camera = cameraTexture.width;
        int height_camera = cameraTexture.height;

        if (needToUpdateIntrinsics)
        {
            XRCameraIntrinsics intrinsic;
            arCameraManager.TryGetIntrinsics(out intrinsic);

            float ratio = (float) width_depth / (float) width_camera;
            fx = intrinsic.focalLength.x * ratio;
            fy = intrinsic.focalLength.y * ratio;

            cx = intrinsic.principalPoint.x * ratio;
            cy = intrinsic.principalPoint.y * ratio;
            needToUpdateIntrinsics = false;
        }

        Color[] depthPixels = depthTextureFloat.GetPixels();
        Color32[] confidenceMap = depthConfidenceTextureR8.GetPixels32(); 
        Matrix4x4 trs = Matrix4x4.TRS(camPosition, camRotation, new Vector3(1, 1, 1));

        int index_dst;
        float depth, u, v;
        float width_height = width_depth * height_depth;
        
        int pointsPerLine = (int) (width_height * maxPointsPerFrame / height_depth);
        float jumpValue = (width_depth / pointsPerLine) - 1f;
        float jumpCounter = jumpValue;
        Vector3 vertice, normal;
        Color color;

        for(int depth_y = 0; depth_y < height_depth; depth_y ++)
        {
            index_dst = depth_y * width_depth;
            v = (float) depth_y / (height_depth) * height_camera;

            for(int depth_x = 0; depth_x < width_depth; depth_x ++)
            {
                if (jumpCounter <= 1f)
                {
                    jumpCounter += jumpValue;
                    depth = depthPixels[index_dst].r;

                    if (depth > near && depth < far && confidenceMap[index_dst].r == 2)
                    {
                        vertice.z = depth;
                        vertice.x = - depth * (depth_x - cx) / fx;
                        vertice.y = - depth * (depth_y - cy) / fy;
                        vertice = trs.MultiplyPoint(vertice);

                        if (vertice == Vector3.zero) 
                        {
                            index_dst ++;
                            continue;
                        }

                        Vector3 chunkPos = new Vector3(Mathf.Round(vertice.x / chunkSize), Mathf.Round(vertice.y / chunkSize), Mathf.Round(vertice.z / chunkSize));
                        if (chunksThatAreFull.Contains(chunkPos))
                        {
                            index_dst ++;
                            continue;
                        }

                        u = (float) depth_x / (width_depth);
                        color = cameraTexture.GetPixel((int) (u * width_camera), (int) v);
                        normal = (mainCamera.transform.position - vertice).normalized;

                        if (pointCloudFrameChunk.ContainsKey(chunkPos))
                        {
                            if (pointCloudFrameChunk[chunkPos].vertices.Count > maxVerticesPerNewMesh)
                            {
                                index_dst ++;
                                continue;
                            }

                            pointCloudFrameChunk[chunkPos].vertices.Add(vertice);
                            pointCloudFrameChunk[chunkPos].normals.Add(normal);
                            pointCloudFrameChunk[chunkPos].colors.Add(color);
                        }

                        else
                        {
                            PointCloudChunk pc = new PointCloudChunk();
                            pc.vertices = new List<Vector3>() { vertice };
                            pc.normals = new List<Vector3>() { normal };
                            pc.colors = new List<Color>() { color };
                            pointCloudFrameChunk.Add(chunkPos, pc);
                        }
                    }
                }

                else jumpCounter -= 1f;
                index_dst ++;
            }
        }

        double timeNow = Time.realtimeSinceStartup;
        Print("First step: " + (timeNow - lastInterval));
        lastInterval = Time.realtimeSinceStartup;
        
        foreach (Vector3 chunk in pointCloudFrameChunk.Keys)
        {
            if (pointCloudChunks.ContainsKey(chunk))
            {
                int verticesCount = pointCloudChunks[chunk].verticesCount + pointCloudFrameChunk[chunk].vertices.Count;
                int idx = pointCloudFrameChunk[chunk].vertices.Count;
                if (verticesCount > maxVerticesPerChunk) 
                    idx -= (verticesCount - maxVerticesPerChunk);

                if (pointCloudChunks[chunk].chunkCount <= maxMeshesPerChunk && idx >= 0)
                {
                    CreatePointCloudMesh(pointCloudFrameChunk[chunk].vertices.GetRange(0, idx).ToArray(), pointCloudFrameChunk[chunk].colors.GetRange(0, idx).ToArray(), pointCloudFrameChunk[chunk].normals.GetRange(0, idx).ToArray(), processedChunks.transform);
                    Chunk c = pointCloudChunks[chunk];
                    c.chunkCount ++;
                    c.verticesCount = verticesCount;

                    pointCloudChunks[chunk] = c;
                    totalMeshes ++;
                    totalPoints += pointCloudFrameChunk[chunk].vertices.Count;
                }

                else
                    chunksThatAreFull.Add(chunk);
            }

            else
            {
                int idx = pointCloudFrameChunk[chunk].vertices.Count;
                if (idx > maxVerticesPerChunk) 
                    idx -= (idx - maxVerticesPerChunk);

                CreatePointCloudMesh(pointCloudFrameChunk[chunk].vertices.GetRange(0, idx).ToArray(), pointCloudFrameChunk[chunk].colors.GetRange(0, idx).ToArray(), pointCloudFrameChunk[chunk].normals.GetRange(0, idx).ToArray(), processedChunks.transform);
                Chunk c = new Chunk();
                c.chunkCount = 1;
                c.verticesCount = idx;

                pointCloudChunks.Add(chunk, c);
                totalMeshes ++;
                totalPoints += idx;
            }
        }

        timeNow = Time.realtimeSinceStartup;
        Print("Second step: " + (timeNow - lastInterval));
    }

    #region Threaded Point Cloud Processing
    void ProcessChunksInThread()
    {
        int verticesCount = 0;

        MeshFilter[] mfs = processedChunks.GetComponentsInChildren<MeshFilter>();
        foreach (MeshFilter mf in mfs)
            verticesCount += mf.mesh.vertices.Length;

        if (verticesCount < verticesPPostProcessedMesh) return;
        foreach (MeshFilter mf in mfs)
        {
            verticesInThread.AddRange(mf.mesh.vertices);
            colorsInThread.AddRange(mf.mesh.colors);
            normalsInThread.AddRange(mf.mesh.normals);
            meshToRemove.Add(mf);
        }

        nMeshesToCreate = Mathf.CeilToInt((float) verticesInThread.Count / verticesPPostProcessedMesh);
        meshesSaved += mfs.Length - nMeshesToCreate;
    }

    void CreateMeshesInThread()
    {
        double lastInterval = Time.realtimeSinceStartup;
        CreateLoadOfMeshes(nMeshesToCreate, verticesInThread, colorsInThread, normalsInThread, postProcessedChunks.transform);
        verticesInThread = new List<Vector3>();
        colorsInThread = new List<Color>();
        normalsInThread = new List<Vector3>();

        for (int i = 0; i < meshToRemove.Count; i ++)
            meshToRemove[i].gameObject.transform.parent = chunksThatNeedToBeDeleted.transform;

        for (int i = 0; i < chunksThatNeedToBeDeleted.transform.childCount; i ++)
            Destroy(chunksThatNeedToBeDeleted.transform.GetChild(i).gameObject);

        nMeshesToCreate = 0;
        meshToRemove = new List<MeshFilter>();
        double timeNow = Time.realtimeSinceStartup;
        Print("Third step: " + (timeNow - lastInterval));
    }
    #endregion
    #endregion

    #region Util Functions
    void SetupScan()
    {
        pointCLoudMaster = new GameObject("pointCLoudMaster");
        processedChunks = new GameObject("processedChunks");
        postProcessedChunks = new GameObject("postProcessedChunks");
        chunksThatNeedToBeDeleted = new GameObject("chunksToDelete");

        processedChunks.transform.parent = pointCLoudMaster.transform;
        postProcessedChunks.transform.parent = pointCLoudMaster.transform;
        chunksThatNeedToBeDeleted.transform.parent = pointCLoudMaster.transform;

        mainCamera = arCameraManager.gameObject.GetComponent<Camera>();
        lastCamPosition = arCameraManager.transform.position;
        lastCamRotation = arCameraManager.transform.rotation;

        if (threadedMeshOptimization)
            thread = new Thread(ProcessChunksInThread);

        pointCloudChunks = new Dictionary<Vector3, Chunk>();
        chunksThatAreFull = new HashSet<Vector3>();
        voxelGridVertices = new List<Vector3>();
        voxelGridNormals = new List<Vector3>();
        voxelGridColors = new List<Color>();
        meshToRemove = new List<MeshFilter>();
        verticesInThread = new List<Vector3>();
        normalsInThread = new List<Vector3>();
        colorsInThread = new List<Color>();
    }

    void CreateLoadOfMeshes(int n, List<Vector3> vertices, List<Color> colors, List<Vector3> normals, Transform parent)
    {
        for (int i = 0; i < n; i ++)
        {
            int start = i * verticesPPostProcessedMesh;
            int end = Mathf.Min(start + verticesPPostProcessedMesh, vertices.Count);
            int count = end - start;

            CreatePointCloudMesh(vertices.GetRange(start, count).ToArray(), colors.GetRange(start, count).ToArray(), normals.GetRange(start, count).ToArray(), parent);
        }
    }

    GameObject CreatePointCloudMesh(Vector3[] vertices, Color[] colors, Vector3[] normals, Transform parent)
    {   
        GameObject pointCloudChunk = new GameObject("PointCloud", typeof(MeshFilter), typeof(MeshRenderer));
        Mesh meshChunk = pointCloudChunk.GetComponent<MeshFilter>().mesh;
        pointCloudChunk.GetComponent<MeshRenderer>().material = pointCloudMaterial;
        pointCloudChunk.transform.parent = parent;

        meshChunk.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        int[] indices = new int[vertices.Length];
        for (int i = 0; i < vertices.Length; i++) indices[i] = i;

        meshChunk.vertices = vertices;
        meshChunk.colors = colors;
        meshChunk.normals = normals;
        meshChunk.SetIndices(indices, MeshTopology.Points, 0);
        meshChunk.Optimize();
        return pointCloudChunk;
    }

    public void CullNormals(bool cull)
    {
        int iEstimate = cull ? 1 : 0;
        pointCloudMaterial.SetInt("cull_normals", (int) iEstimate);
    }
    #endregion

    #region Debug
    public void ResetScan()
    {
        isScanning = false;

        if (thread != null)
            thread.Abort();

        Destroy(pointCLoudMaster);
        SetupScan();
    }

    void Print(string str)
    {
        // print("<--- Debugger ---> " + str + "\n" + Time.realtimeSinceStartup);
    }
    #endregion
}