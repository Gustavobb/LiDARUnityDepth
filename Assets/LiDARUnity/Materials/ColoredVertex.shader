Shader "Unlit/ColoredVertex"
{
	Properties
    {
        point_size("Point Size", Float) = 5.0
        cull_normals("Cull normals", Float) = 0.0
	}

    SubShader
    {
        Pass 
        {
            Tags {"Queue"="Transparent" "IgnoreProjector"="True" "RenderType"="Transparent"}
            Blend SrcAlpha OneMinusSrcAlpha
            Cull Back

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float3 normal : NORMAL;
                float4 color : COLOR;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                float3 normal : NORMAL;
                float4 color : COLOR;
                float size : PSIZE;
            };

            float4x4 depthCameraTUnityWorld;
            float point_size;
            float cull_normals;

            v2f vert(appdata v)
            {
                // Color should be based on pose relative info
                // o.color = mul(depthCameraTUnityWorld, v.vertex);
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.normal = mul(unity_ObjectToWorld, v.normal);
                o.color = v.color;

                float camera_space = UnityObjectToViewPos(v.vertex).z;
                float near_plane = _ProjectionParams.z;
                float p = 1.0 - (- camera_space / near_plane);
                p = clamp(p * point_size, 1.0, point_size);
                o.size = p;

                if (cull_normals > 0.0)
                {
                    float3 dist_to_camera = _WorldSpaceCameraPos - mul(unity_ObjectToWorld, v.vertex);
                    float dot_result = dot(dist_to_camera, o.normal);
                    if (dot_result < 0) o.color.a = 0;
                }   

                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                return i.color;
            }
            ENDCG
        }
    }
}