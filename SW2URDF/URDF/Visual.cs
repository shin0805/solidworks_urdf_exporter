using System.Runtime.Serialization;

namespace SW2URDF.URDF
{
    //The visual element of a link
    [DataContract(IsReference = true, Namespace = "http://schemas.datacontract.org/2004/07/SW2URDF")]
    public class Visual : URDFElement
    {
        [DataMember]
        public readonly Origin Origin;

        [DataMember]
        public readonly Geometry Geometry;

        [DataMember]
        public readonly Material Material;

        public Visual() : base("visual", false)
        {
            Origin = new Origin(false);
            Geometry = new Geometry();
            Material = new Material();

            ChildElements.Add(Origin);
            ChildElements.Add(Geometry);
            ChildElements.Add(Material);
        }

        public override void WriteURDF(System.Xml.XmlWriter writer)
        {
            string filename = Geometry.Mesh.Filename ?? "";
            if (filename.EndsWith(".dae", System.StringComparison.OrdinalIgnoreCase) ||
                filename.EndsWith(".glb", System.StringComparison.OrdinalIgnoreCase))
            {
                // Use the materials embedded in the mesh.
                writer.WriteStartElement("visual");
                Origin.WriteURDF(writer);
                Geometry.WriteURDF(writer);
                writer.WriteEndElement();
                return;
            }

            base.WriteURDF(writer);
        }
    }
}
