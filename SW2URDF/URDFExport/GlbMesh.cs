using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using SharpGLTF.Geometry;
using SharpGLTF.Geometry.VertexTypes;
using SharpGLTF.Materials;
using SharpGLTF.Scenes;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace SW2URDF.URDFExport
{
    internal sealed class GlbMesh
    {
        private const int MaxTrianglesPerChunk = 60000;
        
        private sealed class MaterialEntry
        {
            public double[] Color;
            public MaterialBuilder Material;
        
            public int MaterialIndex;
            public int NextChunkIndex;
            public int TriangleCountInChunk;
        
            public MeshBuilder<VertexPositionNormal, VertexEmpty, VertexEmpty> CurrentMesh;
        }
        
        private readonly List<MeshBuilder<VertexPositionNormal, VertexEmpty, VertexEmpty>> meshes =
            new List<MeshBuilder<VertexPositionNormal, VertexEmpty, VertexEmpty>>();
        
        private readonly List<MaterialEntry> materials =
            new List<MaterialEntry>();
        
        private int triangleCount;

        internal void AddFace(
            float[] triangles,
            float[] normals,
            Matrix<double> transform,
            double[] color)
        {
            if (triangles == null ||
                triangles.Length == 0 ||
                triangles.Length % 9 != 0)
            {
                throw new InvalidDataException(
                    "No valid face tessellation. Resolve the component and rebuild the model before exporting GLB.");
            }

            if (triangles.Any(x => float.IsNaN(x) || float.IsInfinity(x)))
            {
                throw new InvalidDataException(
                    "The face tessellation contains invalid coordinates.");
            }

            color = ColladaMesh.SelectColor(color);
            
            MaterialEntry materialEntry = GetMaterialEntry(color);
            
            Matrix<double> linear = transform.SubMatrix(0, 3, 0, 3);

            Matrix<double> normalTransform = linear.Inverse().Transpose();

            bool mirrored = linear.Determinant() < 0;

            for (int triangle = 0;
                 triangle < triangles.Length;
                 triangle += 9)
            {
                var points = new Vector<double>[3];

                for (int vertex = 0; vertex < 3; vertex++)
                {
                    int offset = triangle + vertex * 3;

                    points[vertex] =
                        transform *
                        DenseVector.OfArray(new double[]
                        {
                            triangles[offset],
                            triangles[offset + 1],
                            triangles[offset + 2],
                            1
                        });
                }

                int[] order = mirrored
                    ? new[] { 0, 2, 1 }
                    : new[] { 0, 1, 2 };

                var u = points[order[1]] - points[order[0]];
                var v = points[order[2]] - points[order[0]];

                Vector<double> flatNormal =
                    DenseVector.OfArray(new[]
                    {
                        u[1] * v[2] - u[2] * v[1],
                        u[2] * v[0] - u[0] * v[2],
                        u[0] * v[1] - u[1] * v[0]
                    });

                if (flatNormal.L2Norm() < 1e-20)
                    continue;

                flatNormal = flatNormal.Normalize(2);

                var vertices = new VertexPositionNormal[3];

                for (int outputVertex = 0;
                     outputVertex < 3;
                     outputVertex++)
                {
                    int vertex = order[outputVertex];
                    int offset = triangle + vertex * 3;

                    Vector<double> normal = flatNormal;

                    if (normals != null &&
                        normals.Length == triangles.Length)
                    {
                        var transformed =
                            normalTransform *
                            DenseVector.OfArray(new double[]
                            {
                                normals[offset],
                                normals[offset + 1],
                                normals[offset + 2]
                            });

                        if (transformed.All(x =>
                                !double.IsNaN(x) &&
                                !double.IsInfinity(x)) &&
                            transformed.L2Norm() > 1e-20)
                        {
                            normal = transformed.Normalize(2);
                        }
                    }

                vertices[outputVertex] =
                    new VertexPositionNormal(
                        (float)points[vertex][0],
                        (float)points[vertex][1],
                        (float)points[vertex][2],
                        (float)normal[0],
                        (float)normal[1],
                        (float)normal[2]);
                }
                
                if (materialEntry.CurrentMesh == null ||
                    materialEntry.TriangleCountInChunk >= MaxTrianglesPerChunk)
                {
                    StartNewChunk(materialEntry);
                }
                
                var primitive =
                    materialEntry.CurrentMesh.UsePrimitive(
                        materialEntry.Material);
                
                primitive.AddTriangle(
                    vertices[0],
                    vertices[1],
                    vertices[2]);
                
                materialEntry.TriangleCountInChunk++;
                triangleCount++;
            }
        }

        internal void Write(string path)
        {
            if (triangleCount == 0)
            {
                throw new InvalidDataException(
                    "The link has no triangles to export as GLB.");
            }
        
            var scene = new SceneBuilder();
        
            foreach (var mesh in meshes)
            {
                scene.AddRigidMesh(
                    mesh,
                    System.Numerics.Matrix4x4.Identity);
            }
        
            scene.ToGltf2().SaveGLB(path);
        }

        private MaterialEntry GetMaterialEntry(double[] color)
        {
            MaterialEntry existing =
                materials.FirstOrDefault(
                    item => item.Color.SequenceEqual(color));
        
            if (existing != null)
                return existing;
        
            int materialIndex = materials.Count;
        
            var material =
                new MaterialBuilder("material" + materialIndex)
                    .WithChannelParam(
                        KnownChannel.BaseColor,
                        KnownProperty.RGBA,
                        new System.Numerics.Vector4(
                            (float)color[0],
                            (float)color[1],
                            (float)color[2],
                            1.0f))
                    .WithDoubleSide(true);
        
            var entry = new MaterialEntry
            {
                Color = color.ToArray(),
                Material = material,
                MaterialIndex = materialIndex,
                NextChunkIndex = 0,
                TriangleCountInChunk = 0,
                CurrentMesh = null
            };
        
            materials.Add(entry);
        
            return entry;
        }
        
        private void StartNewChunk(MaterialEntry entry)
        {
            entry.CurrentMesh =
                new MeshBuilder<VertexPositionNormal, VertexEmpty, VertexEmpty>(
                    "mesh-" +
                    entry.MaterialIndex +
                    "-chunk-" +
                    entry.NextChunkIndex);
        
            entry.NextChunkIndex++;
            entry.TriangleCountInChunk = 0;
        
            meshes.Add(entry.CurrentMesh);
        }
    }
}
