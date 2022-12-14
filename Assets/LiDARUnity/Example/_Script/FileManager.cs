using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Networking;
using System.Threading;
using System.Text;

public class FileManager : MonoBehaviour
{
 public bool doneScanRequest = false;
    PointCloudLiDAR pointCloudLiDAR;
    Thread thread;

    void Start()
    {
        pointCloudLiDAR = FindObjectsOfType<PointCloudLiDAR>()[0];
        thread = new Thread(SavePCD);
    }

    public void SaveScan()
    {
        thread.Start();
    }

    void SavePCD()
    {
        print("Getting PCD");
        MeshFilter[] meshFilters = pointCloudLiDAR.pointCLoudMaster.GetComponentsInChildren<MeshFilter>();

        int totalVertices = 0;
        for (int i = 0; i < meshFilters.Length; i++)
            totalVertices += meshFilters[i].mesh.vertexCount;

        int currentVertices = 0;

        List<Vector3> vertices = new List<Vector3>();
        List<Vector3> normals = new List<Vector3>();
        List<Color> colors = new List<Color>();

        foreach (MeshFilter mf in meshFilters)
        {
            float percentage = (float) currentVertices / (float) totalVertices * 100;

            vertices.AddRange(mf.mesh.vertices);
            colors.AddRange(mf.mesh.colors);
            normals.AddRange(mf.mesh.normals);

            currentVertices += mf.mesh.vertexCount;
            print(percentage);
        }

        PointCloudData pcd = new PointCloudData() 
        {
            vertices = vertices.ToArray(),
            colors = colors.ToArray(),
            normals = normals.ToArray()
        };

        string json = JsonUtility.ToJson(pcd);
        byte[] bytes = Encoding.ASCII.GetBytes(json);

        string filePath = Application.persistentDataPath;
        string fileName = "pcd_" + System.DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss") + ".txt";
        
        // if (System.IO.File.Exists(filePath + "/" + fileName))
        //     System.IO.File.Delete(filePath + "/" + fileName);
        
        // System.IO.File.WriteAllBytes(filePath + "/" + fileName, bytes);
        print("PCD saved");
    }

    void OnApplicationQuit()
    {
        if (thread != null) thread.Abort();
    }
}

[System.Serializable]
public class PointCloudData
{
    public Vector3[] vertices;
    public Vector3[] normals;
    public Color[] colors;
}