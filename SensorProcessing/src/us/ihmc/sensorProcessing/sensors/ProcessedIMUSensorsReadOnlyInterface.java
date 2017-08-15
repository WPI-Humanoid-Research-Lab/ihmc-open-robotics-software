package us.ihmc.sensorProcessing.sensors;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.FrameVector3D;

public interface ProcessedIMUSensorsReadOnlyInterface
{
   public abstract FrameVector3D getAcceleration(int imuIndex);
   
   public abstract double getYaw(int imuIndex);

   public abstract double getPitch(int imuIndex);

   public abstract double getRoll(int imuIndex);
   
   public abstract Quaternion getQuaternion(int imuIndex);

   public abstract FrameVector3D getAngularVelocity(int imuIndex);
   
   public abstract FrameVector3D getAngularAcceleration(int imuIndex);
}