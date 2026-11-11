using Moq;
using SW2URDF.UI;
using SW2URDF.URDFExport;
using System;
using System.IO;
using System.Windows;
using Xunit;

namespace SW2URDF.Test
{
    public class TestMeshDirectory
    {
        [Theory]
        [InlineData("meshes", "meshes")]
        [InlineData(@"assets\robot\meshes", "assets/robot/meshes")]
        [InlineData("assets/robot/meshes/", "assets/robot/meshes")]
        [InlineData("assets\u00a5meshes", "assets/meshes")]
        public void NormalizesRelativePaths(string input, string expected)
        {
            var package = new URDFPackage("robot", Path.GetTempPath(), input);
            Assert.Equal(expected, package.MeshDirectory);
            Assert.Equal("package://robot/" + expected + "/", package.MeshesDirectory);
            Assert.Equal(Path.Combine(Path.GetTempPath(), "robot", expected.Replace('/', '\\')) + "\\", package.WindowsMeshesDirectory);
        }

        [Theory]
        [InlineData("")]
        [InlineData(" ")]
        [InlineData(@"C:\meshes")]
        [InlineData(@"\\server\meshes")]
        [InlineData("/meshes")]
        [InlineData("../meshes")]
        [InlineData("assets/../meshes")]
        [InlineData("assets//meshes")]
        [InlineData("assets/CON")]
        [InlineData("assets/invalid?")]
        [InlineData("assets/trailing.")]
        public void RejectsInvalidPaths(string input)
        {
            Assert.Throws<ArgumentException>(() => new URDFPackage("robot", Path.GetTempPath(), input));
        }

        [Fact]
        public void DefaultRemainsMeshes()
        {
            Assert.Equal("package://robot/meshes/", new URDFPackage("robot", Path.GetTempPath()).MeshesDirectory);
        }

        [Fact]
        public void CreatesNestedFolderAndMatchingInstallRule()
        {
            string directory = Path.Combine(Path.GetTempPath(), Guid.NewGuid().ToString());
            var original = URDFPackage.MessageBox;
            try
            {
                URDFPackage.MessageBox = new Mock<IMessageBox>().Object;
                var package = new URDFPackage("robot", directory, @"assets\robot meshes");
                package.CreateDirectories();
                package.CreateCMakeLists();
                Assert.True(Directory.Exists(Path.Combine(directory, "robot", "assets", "robot meshes")));
                Assert.Equal("package://robot/assets/robot%20meshes/", package.MeshesDirectory);
                string cmake = File.ReadAllText(package.WindowsCMakeLists);
                Assert.Contains("install(DIRECTORY \"assets/robot meshes/\"", cmake);
                Assert.Contains("DESTINATION \"${CATKIN_PACKAGE_SHARE_DESTINATION}/assets/robot meshes\"", cmake);
                Assert.DoesNotContain("foreach(dir config launch meshes urdf)", cmake);
            }
            finally
            {
                URDFPackage.MessageBox = original;
                if (Directory.Exists(directory)) Directory.Delete(directory, true);
            }
        }

        [Theory]
        [InlineData(MeshExportFormat.STL, ".STL")]
        [InlineData(MeshExportFormat.THREEDXML, ".3dxml")]
        [InlineData(MeshExportFormat.DAE, ".dae")]
        [InlineData(MeshExportFormat.GLB, ".glb")]
        public void ExportWithoutMeshesUsesConfiguredReferences(MeshExportFormat format, string extension)
        {
            var exporter = (ExportHelper)System.Runtime.Serialization.FormatterServices.GetUninitializedObject(typeof(ExportHelper));
            var flags = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic;
            typeof(ExportHelper).GetField("progressBar", flags).SetValue(exporter,
                new Mock<SolidWorks.Interop.sldworks.UserProgressBar>().Object);
            var link = new SW2URDF.URDF.Link { Name = "base" };
            var package = new URDFPackage("robot", Path.GetTempPath(), @"assets\meshes");
            typeof(ExportHelper).GetMethod("ExportFiles", flags).Invoke(exporter, new object[] { link, package, 0, false, format });
            Assert.Equal("package://robot/assets/meshes/base" + extension, link.Visual.Geometry.Mesh.Filename);
            Assert.Equal(link.Visual.Geometry.Mesh.Filename, link.Collision.Geometry.Mesh.Filename);
        }
    }

    public class TestURDFPackage : SW2URDFTest
    {
        public TestURDFPackage(SWTestFixture fixture) : base(fixture)
        {
        }

        [Fact]
        public void TestCreateDirectories()
        {
            string tempDirectory = CreateRandomTempDirectory();
            string name = Path.GetRandomFileName();
            URDFPackage pkg = new URDFPackage(name, tempDirectory);

            Mock<IMessageBox> messageBoxMock = new Mock<IMessageBox>();
            messageBoxMock.Setup(m => m.Show(It.IsAny<string>()))
                .Returns(MessageBoxResult.OK); //can be whatever depends on test case
            URDFPackage.MessageBox = messageBoxMock.Object;
            pkg.CreateDirectories();

            Assert.True(Directory.Exists(pkg.WindowsPackageDirectory));
            Assert.True(Directory.Exists(pkg.WindowsMeshesDirectory));
            Assert.True(Directory.Exists(pkg.WindowsRobotsDirectory));
            Assert.True(Directory.Exists(pkg.WindowsTexturesDirectory));
            Assert.True(Directory.Exists(pkg.WindowsLaunchDirectory));
            Assert.True(Directory.Exists(pkg.WindowsConfigDirectory));

            Directory.Delete(tempDirectory, true);
        }

        [Fact]
        public void TestCreateCMakeLists()
        {
            string tempDirectory = CreateRandomTempDirectory();
            string name = Path.GetRandomFileName();
            URDFPackage pkg = new URDFPackage(name, tempDirectory);
            Mock<IMessageBox> messageBoxMock = new Mock<IMessageBox>();
            messageBoxMock.Setup(m => m.Show(It.IsAny<string>()))
                .Returns(MessageBoxResult.OK); //can be whatever depends on test case
            URDFPackage.MessageBox = messageBoxMock.Object;
            pkg.CreateDirectories();
            pkg.CreateCMakeLists();

            Assert.True(File.Exists(pkg.WindowsCMakeLists));

            Console.WriteLine("Deleting directory " + tempDirectory);
            Directory.Delete(tempDirectory, true);
        }

        [Fact]
        public void TestCreateConfigYAML()
        {
            string tempDirectory = CreateRandomTempDirectory();
            string name = Path.GetRandomFileName();
            Directory.CreateDirectory(tempDirectory);
            URDFPackage pkg = new URDFPackage(name, tempDirectory);
            Mock<IMessageBox> messageBoxMock = new Mock<IMessageBox>();
            messageBoxMock.Setup(m => m.Show(It.IsAny<string>()))
                .Returns(MessageBoxResult.OK); //can be whatever depends on test case
            URDFPackage.MessageBox = messageBoxMock.Object;
            pkg.CreateDirectories();
            pkg.CreateConfigYAML(new string[] {"a", "b", "c"});

            Assert.True(File.Exists(pkg.WindowsConfigYAML));

            Console.WriteLine("Deleting directory " + tempDirectory);
            Directory.Delete(tempDirectory, true);
        }
    }
}
