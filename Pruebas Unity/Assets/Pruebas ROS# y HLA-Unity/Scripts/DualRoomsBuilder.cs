using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Genera dinámicamente la geometría del mundo de Gazebo en Unity,
/// leyendo el fichero JSON exportado por sdf_to_json.py.
///
/// Flujo:
///   1. Gazebo launch → sdf_to_json.py → /tmp/dualsim_world.json
///   2. Unity Play → este script lee el JSON → crea cubos con las
///      mismas posiciones y tamaños que el SDF de Gazebo.
///
/// Mapeo de coordenadas (Gazebo → Unity):
///   Gazebo X → Unity X
///   Gazebo Y → Unity Z
///   Gazebo Z → Unity Y
///
/// USO: Adjuntar a un GameObject vacío en la escena.
///      Lanzar Gazebo ANTES de darle Play a Unity.
/// </summary>
public class DualRoomsBuilder : MonoBehaviour
{
    [Header("Configuración")]
    [Tooltip("Ruta al fichero JSON generado por Gazebo")]
    public string jsonPath = "/tmp/dualsim_world.json";

    [Tooltip("Offset Y del suelo en Unity (0 si el suelo está en Y=0)")]
    public float groundY = 0.0f;

    [Tooltip("Color por defecto si el JSON no especifica uno")]
    public Color defaultColor = new Color(0.7f, 0.7f, 0.7f, 1.0f);

    // ── Clases para deserializar el JSON ──

    [System.Serializable]
    private class WorldData
    {
        public string source_file;
        public string world_name;
        public int model_count;
        public ModelData[] models;
    }

    [System.Serializable]
    private class ModelData
    {
        public string name;
        public PoseData pose;
        public SizeData size;
        public ColorData color;
    }

    [System.Serializable]
    private class PoseData
    {
        public float x, y, z;
        public float roll, pitch, yaw;
    }

    [System.Serializable]
    private class SizeData
    {
        public float x, y, z;
    }

    [System.Serializable]
    private class ColorData
    {
        public float r, g, b, a;
    }

    void Start()
    {
        if (!System.IO.File.Exists(jsonPath))
        {
            Debug.LogWarning($"[WorldBuilder] No se encontró '{jsonPath}'. " +
                           "Lanza Gazebo antes de darle Play a Unity.");
            return;
        }

        string jsonContent = System.IO.File.ReadAllText(jsonPath);
        WorldData world = JsonUtility.FromJson<WorldData>(jsonContent);

        if (world == null || world.models == null || world.models.Length == 0)
        {
            Debug.Log($"[WorldBuilder] Mundo '{world?.world_name ?? "?"}' sin modelos estáticos. " +
                     "Escena vacía.");
            return;
        }

        Debug.Log($"[WorldBuilder] Cargando mundo '{world.world_name}' " +
                  $"({world.model_count} modelos) desde {world.source_file}");

        // Contenedor para todos los objetos generados
        GameObject container = new GameObject($"World_{world.world_name}");
        container.transform.SetParent(transform);

        int created = 0;
        foreach (var model in world.models)
        {
            if (model.size == null || model.pose == null)
            {
                Debug.LogWarning($"[WorldBuilder] Modelo '{model.name}' sin pose/size, saltando.");
                continue;
            }

            CreateBox(container.transform, model);
            created++;
        }

        Debug.Log($"[WorldBuilder] Generados {created} objetos del mundo '{world.world_name}'.");
    }

    void CreateBox(Transform parent, ModelData model)
    {
        GameObject box = GameObject.CreatePrimitive(PrimitiveType.Cube);
        box.name = model.name;
        box.transform.SetParent(parent);

        // Mapeo de posición Gazebo → Unity:
        //   Gazebo X → Unity X
        //   Gazebo Y → Unity Z
        //   Gazebo Z → Unity Y
        box.transform.position = new Vector3(
            model.pose.x,                     // X = X
            model.pose.z + groundY,           // Y = gz (altura) + offset suelo
            model.pose.y                      // Z = gy
        );

        // Mapeo de tamaño Gazebo → Unity:
        //   Gazebo sx → Unity sx (ancho)
        //   Gazebo sy → Unity sz (profundidad)
        //   Gazebo sz → Unity sy (alto)
        box.transform.localScale = new Vector3(
            model.size.x,                     // ancho
            model.size.z,                     // alto
            model.size.y                      // profundidad
        );

        // Rotación: si hay yaw (rotación en Z de Gazebo → Y de Unity)
        if (model.pose.yaw != 0f)
        {
            box.transform.rotation = Quaternion.Euler(
                -model.pose.pitch * Mathf.Rad2Deg,  // pitch
                model.pose.yaw * Mathf.Rad2Deg,     // yaw
                model.pose.roll * Mathf.Rad2Deg      // roll
            );
        }

        // Material con color del JSON
        Color color = defaultColor;
        if (model.color != null)
        {
            color = new Color(model.color.r, model.color.g, model.color.b, model.color.a);
        }

        Material mat = new Material(Shader.Find("Standard"));
        mat.color = color;
        box.GetComponent<Renderer>().material = mat;

        // Marcar como estático
        box.isStatic = true;
    }
}
