using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

public class UIManager : MonoBehaviour
{
    [SerializeField] GameObject settings, downSampling1, downSampling2;
    [SerializeField] Text fps, pointSizeText, chunkSizeText, maxPointsPerFrameText, maxVerticesPerChunkText, maxVerticesPerNewMeshText, maxMeshesPerChunkText, frameWaitText, farText, divideChunkIntoText, minChunkNeededText; // meshesSavedText, totalMeshesText, totalPointsText, realTotalMeshesText;
    [SerializeField] RawImage cameraView, grayDepthView, confidenceView;
    [SerializeField] Material pointCloudMaterial;
    [SerializeField] PointCloudLiDAR pointCloudLiDAR;

    float frameCount = 0f;
    float dt = 0.0f;
    float fpsValue = 0.0f;
    float updateRate = 4.0f;

    #region Unity Functions
    public void Start()
    {
        pointCloudLiDAR.onPointCloudImagesReady += UpdateUIImages;
        chunkSizeText.text = pointCloudLiDAR.chunkSize.ToString();
        maxPointsPerFrameText.text = pointCloudLiDAR.maxPointsPerFrame.ToString();
        maxVerticesPerChunkText.text = pointCloudLiDAR.maxVerticesPerChunk.ToString();
        maxVerticesPerNewMeshText.text = pointCloudLiDAR.maxVerticesPerNewMesh.ToString();
        maxMeshesPerChunkText.text = pointCloudLiDAR.maxMeshesPerChunk.ToString();
        frameWaitText.text = pointCloudLiDAR.frameWait.ToString();
        farText.text = pointCloudLiDAR.far.ToString();
        divideChunkIntoText.text = pointCloudLiDAR.divideChunkInto.ToString();
        minChunkNeededText.text = pointCloudLiDAR.numberOfReadyChunksNeededToPostProcess.ToString();
        pointCloudLiDAR.CullNormals(true);
        pointCloudLiDAR.fastNormalEstimation = false;
        // pointCloudLiDAR.showNormals = true;
    }

    void Update()
    {
        frameCount++;
        dt += Time.deltaTime;

        if (dt > 1.0f/updateRate)
        {
            fpsValue = frameCount / dt ;
            frameCount = 0f;
            dt -= 1.0f/updateRate;

            fps.text = fpsValue.ToString("F2") + " FPS";
        }
    }
    #endregion

    #region UI
    public void UpdateUIImages()
    {
        cameraView.texture = pointCloudLiDAR.cameraTexture;
        grayDepthView.texture = pointCloudLiDAR.depthTextureBGRA;
        confidenceView.texture = pointCloudLiDAR.depthConfidenceTextureRGBA;
    }

    public void SwitchScanMode(bool flg)
    {
        pointCloudLiDAR.frameCount = 0;
        pointCloudLiDAR.isScanning = flg;
    }

    public void UseDownSampling(bool flg)
    {
        pointCloudLiDAR.useVoxelDownSampling = flg;
        downSampling1.SetActive(flg);
        downSampling2.SetActive(flg);
    }

    public void ToggleImages(bool flg)
    {
        cameraView.gameObject.SetActive(flg);
        grayDepthView.gameObject.SetActive(flg);
        confidenceView.gameObject.SetActive(flg);
    }

    public void ReloadScene()
    {
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
    }

    public void UpdateChunkSizeWithSlider(float value)
    {
        pointCloudLiDAR.chunkSize = value;
        chunkSizeText.text = pointCloudLiDAR.chunkSize.ToString();
    }

    public void UpdateMaxPointsPerFrameWithSlider(float value)
    {
        pointCloudLiDAR.maxPointsPerFrame = value;
        maxPointsPerFrameText.text = pointCloudLiDAR.maxPointsPerFrame.ToString();
    }

    public void UpdateMaxVerticesPerChunkWithSlider(float value)
    {
        pointCloudLiDAR.maxVerticesPerChunk = (int) value;
        maxVerticesPerChunkText.text = pointCloudLiDAR.maxVerticesPerChunk.ToString();
    }

    public void UpdateMaxVerticesPerNewMeshWithSlider(float value)
    {
        pointCloudLiDAR.maxVerticesPerNewMesh = (int) value;
        maxVerticesPerNewMeshText.text = pointCloudLiDAR.maxVerticesPerNewMesh.ToString();
    }

    public void UpdateFrameWaitWithSlider(float value)
    {
        pointCloudLiDAR.frameWait = (int) value;
        frameWaitText.text = pointCloudLiDAR.frameWait.ToString();
    }

    public void UpdateDivideChunkIntoWithSlider(float value)
    {
        pointCloudLiDAR.divideChunkInto = (int) value;
        divideChunkIntoText.text = pointCloudLiDAR.divideChunkInto.ToString();
    }

    public void UpdateMinChunkNeededWithSlider(float value)
    {
        pointCloudLiDAR.numberOfReadyChunksNeededToPostProcess = (int) value;
        minChunkNeededText.text = pointCloudLiDAR.numberOfReadyChunksNeededToPostProcess.ToString();
    }

    public void UpdateMaxMeshPerChunkWithSlider(float value)
    {
        pointCloudLiDAR.maxMeshesPerChunk = (int) value;
        maxMeshesPerChunkText.text = pointCloudLiDAR.maxMeshesPerChunk.ToString();
    }

    public void UpdateFarWithSlider(float value)
    {
        pointCloudLiDAR.far = value;
        farText.text = pointCloudLiDAR.far.ToString();
    }
    
     public void ToggleSettings(bool flg)
    {
        settings.SetActive(flg);
    }

    public void ToggleTresh(bool flg)
    {
        pointCloudLiDAR.useTresh = flg;
    }

    public void ToggleTread(bool flg)
    {
        pointCloudLiDAR.threadedMeshOptimization = flg;
    }

    public void UpdatePointSizeWithSlider(float value)
    {
        pointCloudMaterial.SetInt("point_size", (int) value);
        pointSizeText.text = value.ToString();
    }
    #endregion
}
