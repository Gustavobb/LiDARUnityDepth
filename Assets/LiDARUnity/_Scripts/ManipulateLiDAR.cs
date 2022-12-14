using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.ARFoundation;
using UnityEngine.XR.ARSubsystems;

public class ManipulateLiDAR : MonoBehaviour
{
    #region Variables
    Camera cameraMain;
    PointCloudLiDAR pointCloudLiDAR;
    ARSession arSession;
    GameObject centerOfMass, target;
    float zoomSpeed = 50f, speed = 100f, deltaDistance = 0f, smoothTime = .3f;
    Vector3 targetVelocity = Vector3.zero;
    Vector2 deltaTouch = Vector2.zero;
    Vector2 deltaTranslation = Vector2.zero;
    #endregion

    #region Unity Methods
    void Start()
    {
        pointCloudLiDAR = FindObjectsOfType<PointCloudLiDAR>()[0];
        arSession = FindObjectOfType<ARSession>();
    }

    void Update()
    {
        if (target == null) return;
        HandleTouch();
        HandleTransform();
    }
    #endregion

    #region UI Methods
    public void VisualizeScan()
    {
        pointCloudLiDAR.isScanning = false;
        FindPCDCenterOfMass();
        HandlePCDInteraction();
    }

    public void ResetManipulation()
    {
        Destroy(centerOfMass);
        target = null;
        centerOfMass = null;
        arSession.gameObject.SetActive(true);
    }
    #endregion

    #region Pre Process Data
    void HandlePCDInteraction()
    {
        arSession.gameObject.SetActive(false);
        cameraMain = pointCloudLiDAR.mainCamera;
        target = centerOfMass;
    }

    void FindPCDCenterOfMass()
    {
        MeshRenderer[] mr = pointCloudLiDAR.pointCLoudMaster.GetComponentsInChildren<MeshRenderer>();
        Vector3 minBounds = mr[0].bounds.min;
        Vector3 maxBounds = mr[0].bounds.max;

        foreach (MeshRenderer meshRenderer in mr)
        {
            if (meshRenderer.bounds.min.x < minBounds.x)
                minBounds.x = meshRenderer.bounds.min.x;
            else if (meshRenderer.bounds.max.x > maxBounds.x)
                maxBounds.x = meshRenderer.bounds.max.x;

            if (meshRenderer.bounds.min.y < minBounds.y)
                minBounds.y = meshRenderer.bounds.min.y;
            else if (meshRenderer.bounds.max.y > maxBounds.y)
                maxBounds.y = meshRenderer.bounds.max.y;

            if (meshRenderer.bounds.min.z < minBounds.z)
                minBounds.z = meshRenderer.bounds.min.z;
            else if (meshRenderer.bounds.max.z > maxBounds.z)
                maxBounds.z = meshRenderer.bounds.max.z;
        }

        float centerX = (maxBounds.x + minBounds.x) / 2;
        float centerY = (maxBounds.y + minBounds.y) / 2;
        float centerZ = (maxBounds.z + minBounds.z) / 2;

        centerOfMass = new GameObject();
        centerOfMass.transform.position = new Vector3(centerX, centerY, centerZ);
        pointCloudLiDAR.pointCLoudMaster.transform.parent = centerOfMass.transform;
    }
    #endregion

    #region Handlers
    public void HandleTouch()
    {
        if (Input.touchCount == 1)
        {
            Touch touch = Input.GetTouch(0);
            deltaTouch = touch.deltaPosition * speed / 20f * Time.deltaTime;
        }

        else if (Input.touchCount == 2)
        {
            Touch tZero = Input.GetTouch(0);
            Touch tOne = Input.GetTouch(1);

            float oldTouchDistance = Vector2.Distance(tZero.position - tZero.deltaPosition, tOne.position - tOne.deltaPosition);
            float currentTouchDistance = Vector2.Distance(tZero.position, tOne.position);
            deltaDistance = oldTouchDistance - currentTouchDistance;

            if (Mathf.Abs(deltaDistance) <= 5f)
            {
                deltaDistance = 0f;
                deltaTranslation = Input.GetTouch(0).deltaPosition * speed * 2f * Time.deltaTime;
            }
        }
    }

    void HandleTransform()
    {
        #region General
        Vector3 targetPos = target.transform.position + cameraMain.transform.rotation * new Vector3(deltaTranslation.x, deltaTranslation.y, deltaDistance * zoomSpeed) * Time.deltaTime;
        deltaTranslation = Vector2.zero;
        #endregion

        #region Zoom
        float minDistanceZ = 3f;
        float maxDistanceZ = cameraMain.farClipPlane - 1f;

        Vector3 normalizedTargetPos = cameraMain.transform.InverseTransformPoint(targetPos);
        float clampZ = Mathf.Clamp(normalizedTargetPos.z, minDistanceZ, maxDistanceZ);

        targetPos = cameraMain.transform.TransformPoint(new Vector3(normalizedTargetPos.x, normalizedTargetPos.y, clampZ));
        deltaDistance = 0f;
        #endregion

        #region Rotation
        Vector3 move = cameraMain.transform.rotation *  new Vector3(deltaTouch.y, -deltaTouch.x, 0f);
        Quaternion targetRotation = Quaternion.Euler(move) * target.transform.rotation;
        deltaTouch = Vector2.zero;
        #endregion

        target.transform.rotation = targetRotation;
        target.transform.position = Vector3.SmoothDamp(target.transform.position, targetPos, ref targetVelocity, smoothTime);
    }
    #endregion
}