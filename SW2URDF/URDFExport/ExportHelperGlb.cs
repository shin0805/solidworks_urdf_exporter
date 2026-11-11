using MathNet.Numerics.LinearAlgebra;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SW2URDF.URDF;
using SW2URDF.Utilities;
using System;
using System.Collections.Generic;
using System.IO;

namespace SW2URDF.URDFExport
{
    public partial class ExportHelper
    {
        private void SaveGlb(Link link, string filename)
        {
            MathTransform frame =
                GetCoordinateSystemTransform(
                    link.Joint.CoordinateSystemName);

            if (frame == null)
            {
                throw new InvalidOperationException(
                    "Cannot find the GLB link coordinate system: " +
                    link.Joint.CoordinateSystemName);
            }

            Matrix<double> assemblyToLink =
                MathOps.GetTransformation(frame).Inverse();

            var mesh = new GlbMesh();

            var visited =
                new HashSet<string>(StringComparer.Ordinal);

            CommonSwOperations.ShowComponents(
                ActiveSWModel,
                link.SWComponents);

            try
            {
                ActiveSWModel.GraphicsRedraw2();

                foreach (Component2 component in link.SWComponents)
                {
                    AddGlbComponent(
                        mesh,
                        component,
                        assemblyToLink,
                        visited);
                }
            }
            finally
            {
                CommonSwOperations.HideComponents(
                    ActiveSWModel,
                    link.SWComponents);
            }

            string temporary =
                filename + "." +
                Guid.NewGuid().ToString("N") +
                ".tmp.glb";

            try
            {
                mesh.Write(temporary);

                if (File.Exists(filename))
                    File.Replace(temporary, filename, null);
                else
                    File.Move(temporary, filename);
            }
            finally
            {
                if (File.Exists(temporary))
                    File.Delete(temporary);
            }
        }

        private static void AddGlbComponent(
            GlbMesh mesh,
            Component2 component,
            Matrix<double> assemblyToLink,
            HashSet<string> visited)
        {
            if (component.IsSuppressed() ||
                !visited.Add(component.Name2))
            {
                return;
            }

            var document =
                (ModelDoc2)component.GetModelDoc2();

            if (document == null)
            {
                throw new InvalidOperationException(
                    "Resolve component '" +
                    component.Name2 +
                    "' before exporting GLB.");
            }

            var children =
                (object[])component.GetChildren();

            if (children != null &&
                children.Length > 0)
            {
                foreach (Component2 child in children)
                {
                    AddGlbComponent(
                        mesh,
                        child,
                        assemblyToLink,
                        visited);
                }

                return;
            }

            if (document.GetType() !=
                (int)swDocumentTypes_e.swDocPART)
            {
                return;
            }

            if (component.Transform2 == null)
            {
                throw new InvalidOperationException(
                    "Cannot obtain the transform of " +
                    component.Name2);
            }

            Matrix<double> transform =
                assemblyToLink *
                MathOps.GetTransformation(
                    component.Transform2);

            double[] componentColor = null;

            for (Component2 parent = component;
                 parent != null;
                 parent = parent.GetParent())
            {
                var color =
                    (double[])parent.GetMaterialPropertyValues2(
                        (int)swInConfigurationOpts_e
                            .swThisConfiguration,
                        null);

                if (color != null &&
                    color.Length >= 3 &&
                    color[0] >= 0 &&
                    color[1] >= 0 &&
                    color[2] >= 0)
                {
                    componentColor = color;
                }
            }

            var partColor =
                (double[])document.Extension
                    .GetMaterialPropertyValues(
                        (int)swInConfigurationOpts_e
                            .swSpecifyConfiguration,
                        new[]
                        {
                            component.ReferencedConfiguration
                        });

            foreach (int bodyType in new[]
            {
                (int)swBodyType_e.swSolidBody,
                (int)swBodyType_e.swSheetBody
            })
            {
                var bodies =
                    (object[])component.GetBodies3(
                        bodyType,
                        out _);

                if (bodies == null)
                    continue;

                foreach (Body2 body in bodies)
                {
                    var faces =
                        (object[])body.GetFaces();

                    if (faces == null)
                        continue;

                    var bodyColor =
                        (double[])body.MaterialPropertyValues2;

                    foreach (Face2 face in faces)
                    {
                        var feature =
                            (Feature)face.GetFeature();

                        var featureColor =
                            feature == null
                                ? null
                                : (double[])feature
                                    .GetMaterialPropertyValues2(
                                        (int)swInConfigurationOpts_e
                                            .swSpecifyConfiguration,
                                        new[]
                                        {
                                            component
                                                .ReferencedConfiguration
                                        });

                        double[] color =
                            ColladaMesh.SelectColor(
                                componentColor,
                                (double[])face
                                    .GetMaterialPropertyValues2(
                                        (int)swInConfigurationOpts_e
                                            .swSpecifyConfiguration,
                                        new[]
                                        {
                                            component
                                                .ReferencedConfiguration
                                        }),
                                featureColor,
                                bodyColor,
                                partColor);

                        mesh.AddFace(
                            (float[])face
                                .GetTessTriangles(true),
                            (float[])face.GetTessNorms(),
                            transform,
                            color);
                    }
                }
            }
        }
    }
}
