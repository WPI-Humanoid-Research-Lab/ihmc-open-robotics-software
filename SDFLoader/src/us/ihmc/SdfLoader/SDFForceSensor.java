package us.ihmc.SdfLoader;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;

public class SDFForceSensor
{
   private final String name;
   private final RigidBodyTransform transform;

   public String getName()
   {
      return name;
   }

   public RigidBodyTransform getTransform()
   {
      return transform;
   }

   public SDFForceSensor(String name, RigidBodyTransform transform)
   {
      this.name = name;
      this.transform = transform;
   }

}
