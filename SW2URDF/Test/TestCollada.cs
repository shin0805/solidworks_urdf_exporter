using MathNet.Numerics.LinearAlgebra.Double;
using Moq;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SW2URDF.URDF;
using SW2URDF.URDFExport;
using SW2URDF.Utilities;
using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using Xunit;
using PrivateObject = Microsoft.VisualStudio.TestTools.UnitTesting.PrivateObject;

namespace SW2URDF.Test
{
    public class TestCollada
    {
        private static readonly XNamespace Ns = "http://www.collada.org/2005/11/COLLADASchema";
        private static readonly float[] Triangle = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
        private static double[] Numbers(XElement element) => element.Value.Split(new[] { ' ', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries)
            .Select(x => double.Parse(x, CultureInfo.InvariantCulture)).ToArray();

        [Theory]
        [InlineData(false)]
        [InlineData(true)]
        public void WritesTransformedColoredTrianglesAndValidBindings(bool mirrored)
        {
            string path = Path.GetTempFileName();
            var culture = System.Threading.Thread.CurrentThread.CurrentCulture;
            try
            {
                System.Threading.Thread.CurrentThread.CurrentCulture = new CultureInfo("de-DE");
                var mesh = new ColladaMesh();
                var transform = MathOps.GetTranslation(new double[] { 0.1, 0.2, 0.3 });
                if (mirrored) transform[0, 0] = -1;
                mesh.AddFace(Triangle, null, transform, new double[] { 1, 0, 0 });
                mesh.AddFace(Triangle, null, transform, new double[] { 0, 0, 1 });
                mesh.AddFace(Triangle, null, transform, new double[] { 1, 0, 0 });
                mesh.Write(path);
                var doc = XDocument.Load(path);
                Assert.Equal("1.4.1", (string)doc.Root.Attribute("version"));
                Assert.Equal("1", (string)doc.Descendants(Ns + "unit").Single().Attribute("meter"));
                Assert.Equal("Z_UP", doc.Descendants(Ns + "up_axis").Single().Value);
                Assert.Equal(2, doc.Descendants(Ns + "geometry").Count());
                Assert.Equal(new[] { "1 0 0 1", "0 0 1 1" }, doc.Descendants(Ns + "color").Select(x => x.Value).ToArray());
                Assert.Equal(3, doc.Descendants(Ns + "triangles").Sum(x => (int)x.Attribute("count")));
                foreach (var source in doc.Descendants(Ns + "source"))
                {
                    var array = source.Element(Ns + "float_array");
                    Assert.Equal((int)array.Attribute("count"), Numbers(array).Length);
                    Assert.Equal(Numbers(array).Length / 3, (int)source.Descendants(Ns + "accessor").Single().Attribute("count"));
                }
                double[] positions = Numbers(doc.Descendants(Ns + "float_array").First());
                Assert.Equal(new double[] { 0.1, 0.2, 0.3 }, positions.Take(3).ToArray());
                var a = positions.Skip(3).Take(3).Zip(positions.Take(3), (x, y) => x - y).ToArray();
                var b = positions.Skip(6).Take(3).Zip(positions.Take(3), (x, y) => x - y).ToArray();
                Assert.True(a[0] * b[1] - a[1] * b[0] > 0); // Winding stays aligned with outward normal.
                var normals = Numbers(doc.Descendants(Ns + "float_array").Skip(1).First());
                Assert.Equal(new double[] { 0, 0, 1 }, normals.Take(3).ToArray());
                var ids = doc.Descendants().Attributes("id").Select(x => x.Value).ToArray();
                Assert.Equal(ids.Length, ids.Distinct().Count());
                foreach (var reference in doc.Descendants().Attributes().Where(x => x.Value.StartsWith("#")))
                    Assert.Contains(reference.Value.Substring(1), ids);
                foreach (var triangles in doc.Descendants(Ns + "triangles"))
                {
                    Assert.Equal((int)triangles.Attribute("count") * 3, Numbers(triangles.Element(Ns + "p")).Length);
                    Assert.Contains(doc.Descendants(Ns + "instance_material"),
                        x => (string)x.Attribute("symbol") == (string)triangles.Attribute("material"));
                }
            }
            finally { System.Threading.Thread.CurrentThread.CurrentCulture = culture; File.Delete(path); }
        }

        [Fact]
        public void ColorPrioritySkipsMissingAndInvalidValues()
        {
            Assert.Equal(new double[] { 1, 0, 0 }, ColladaMesh.SelectColor(null, new double[] { -1, -1, -1 }, new double[] { 1, 0, 0 }, new double[] { 0, 0, 1 }));
            Assert.Equal(new double[] { 0.75, 0.75, 0.75 }, ColladaMesh.SelectColor(new double[] { double.NaN, 0, 0 }));
        }

        [Fact]
        public void RejectsMissingGeometryRatherThanWritingEmptyDae()
        {
            var mesh = new ColladaMesh();
            Assert.Throws<InvalidDataException>(() => mesh.AddFace(null, null, DenseMatrix.CreateIdentity(4), null));
            Assert.Throws<InvalidDataException>(() => mesh.Write("unused.dae"));
        }

        [Theory]
        [InlineData(false, false)]
        [InlineData(true, false)]
        [InlineData(false, true)]
        [InlineData(true, true)]
        public void DaeExportUsesLinkCoordinatesAndPreservesInertial(bool componentOverride, bool nested)
        {
            var helper = (ExportHelper)System.Runtime.Serialization.FormatterServices.GetUninitializedObject(typeof(ExportHelper));
            var assembly = new Mock<ModelDoc2>(); var assemblyExtension = new Mock<ModelDocExtension>();
            var selection = new Mock<SelectionMgr>();
            selection.Setup(x => x.CreateSelectData()).Returns(new Mock<SelectData>().Object);
            assembly.SetupGet(x => x.SelectionManager).Returns(selection.Object);
            assembly.SetupGet(x => x.Extension).Returns(assemblyExtension.Object);
            var frame = new Mock<MathTransform>();
            frame.SetupGet(x => x.ArrayData).Returns(new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0 });
            assemblyExtension.Setup(x => x.GetCoordinateSystemTransformByName("frame")).Returns(frame.Object);
            var part = new Mock<ModelDoc2>(); var partExtension = new Mock<ModelDocExtension>();
            part.Setup(x => x.GetType()).Returns((int)swDocumentTypes_e.swDocPART);
            part.SetupGet(x => x.Extension).Returns(partExtension.Object);
            var component = new Mock<Component2>();
            component.SetupGet(x => x.Name2).Returns("part-1");
            component.Setup(x => x.GetModelDoc2()).Returns(part.Object);
            var placement = new Mock<MathTransform>();
            placement.SetupGet(x => x.ArrayData).Returns(new double[] { 0, 1, 0, -1, 0, 0, 0, 0, 1, 2, 0, 0, 1, 0, 0, 0 });
            component.SetupGet(x => x.Transform2).Returns(placement.Object);
            if (componentOverride) component.Setup(x => x.GetMaterialPropertyValues2(It.IsAny<int>(), It.IsAny<object>())).Returns(new double[] { 0, 0, 1 });
            var body = new Mock<Body2>(); var face = new Mock<Face2>();
            object info = null;
            component.Setup(x => x.GetBodies3((int)swBodyType_e.swSolidBody, out info)).Returns(new object[] { body.Object });
            body.Setup(x => x.GetFaces()).Returns(new object[] { face.Object });
            face.Setup(x => x.GetTessTriangles(true)).Returns(Triangle);
            face.Setup(x => x.GetMaterialPropertyValues2((int)swInConfigurationOpts_e.swSpecifyConfiguration, It.IsAny<object>()))
                .Returns(new double[] { 1, 0, 0 });
            helper.ActiveSWModel = assembly.Object;
            var link = new Link { Name = "test" };
            link.Joint.CoordinateSystemName = "frame";
            link.SWComponents.Add(component.Object); link.SWComponents.Add(component.Object); // No duplicate mesh.
            if (nested)
            {
                var parent = new Mock<Component2>();
                var parentDocument = new Mock<ModelDoc2>();
                parent.SetupGet(x => x.Name2).Returns("assembly-1");
                parent.Setup(x => x.GetModelDoc2()).Returns(parentDocument.Object);
                parent.Setup(x => x.GetChildren()).Returns(new object[] { component.Object });
                component.Setup(x => x.GetParent()).Returns(parent.Object);
                // Selecting both a subassembly and its child must not duplicate geometry,
                // and the child's root-relative Transform2 must not be multiplied twice.
                link.SWComponents.Insert(0, parent.Object);
            }
            link.Inertial.Mass.Value = 3;
            link.Inertial.Origin.SetXYZ(new double[] { 0.1, 0.2, 0.3 });
            link.Inertial.Inertia.SetMomentMatrix(new double[] { 1, 0.1, 0.2, 0.1, 2, 0.3, 0.2, 0.3, 3 });
            double[] inertia = link.Inertial.Inertia.GetMoment();
            string path = Path.GetTempFileName();
            try
            {
                new PrivateObject(helper).Invoke("SaveDae", link, path);
                var doc = XDocument.Load(path);
                Assert.Equal(new double[] { 1, 0, 0, 1, 1, 0, 0, 0, 0 }, Numbers(doc.Descendants(Ns + "float_array").First()));
                Assert.Equal(componentOverride ? "0 0 1 1" : "1 0 0 1", doc.Descendants(Ns + "color").Single().Value);
                Assert.Equal(3, link.Inertial.Mass.Value);
                Assert.Equal(new double[] { 0.1, 0.2, 0.3 }, link.Inertial.Origin.GetXYZ());
                Assert.Equal(inertia, link.Inertial.Inertia.GetMoment());
                Assert.Equal(new double[3], link.Visual.Origin.GetXYZ());
            }
            finally { File.Delete(path); }
        }

        [Theory]
        [InlineData(".dae", false)]
        [InlineData(".glb", false)]
        [InlineData(".GLB", false)]
        [InlineData(".STL", true)]
        public void EmbeddedColorsAreNotOverriddenByUrdfMaterial(string extension, bool hasMaterial)
        {
            var visual = new Visual(); visual.Geometry.Mesh.Filename = "package://robot/meshes/link" + extension;
            var builder = new System.Text.StringBuilder();
            using (var writer = System.Xml.XmlWriter.Create(builder)) visual.WriteURDF(writer);
            var xml = XDocument.Parse(builder.ToString());
            Assert.Equal(hasMaterial, xml.Root.Element("material") != null);
        }
    }
}
