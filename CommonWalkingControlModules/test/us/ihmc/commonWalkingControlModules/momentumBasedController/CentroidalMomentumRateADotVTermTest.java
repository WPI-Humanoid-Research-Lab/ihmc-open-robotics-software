package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.CenterOfMassReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.utilities.screwTheory.CentroidalMomentumRateADotVTerm;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTestTools;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.test.JUnitTools;

import com.yobotics.simulationconstructionset.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import com.yobotics.simulationconstructionset.UnreasonableAccelerationException;

public class CentroidalMomentumRateADotVTermTest
{

   private static final double EPSILON = 1.0e-5;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final int NUMBER_OF_ITERATIONS = 5;

   private final double controlDT = 0.00000005;

   private final DenseMatrix64F a = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F aPrevVal = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F aDot = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F aDotVNumerical = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F aDotVAnalytical = new DenseMatrix64F(6, 1);

//   @Ignore
   @Test
   public void chainTest() throws UnreasonableAccelerationException
   {
      Random random = new Random(12651L);

      ArrayList<RevoluteJoint> joints = new ArrayList<>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      int numberOfJoints = 10;
      Vector3d[] jointAxes = new Vector3d[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointAxes[i] = RandomTools.generateRandomVector(random, 1.0);

      ScrewTestTools.createRandomChainRobot("blop", joints, elevator, jointAxes, random);
      SCSRobotFromInverseDynamicsRobotModel robot = new SCSRobotFromInverseDynamicsRobotModel("robot", elevator.getChildrenJoints().get(0));

      assertADotV(random, joints, elevator, robot,numberOfJoints);
   }

//   @Ignore
   @Test
   public void treeTest() throws UnreasonableAccelerationException
   {
      Random random = new Random(12651L);

      ArrayList<RevoluteJoint> joints = new ArrayList<>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      RevoluteJoint rootJoint = ScrewTestTools.addRandomRevoluteJoint("rootJoint", random, elevator); // Just to make sure there is only one root joint for the SCS robot
      RigidBody rootBody = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);

      int numberOfJoints = 10; 
      ScrewTestTools.createRandomTreeRobot(joints, rootBody, numberOfJoints - 1, random);
      joints.add(0, rootJoint);
      SCSRobotFromInverseDynamicsRobotModel robot = new SCSRobotFromInverseDynamicsRobotModel("robot", rootJoint);

      assertADotV(random, joints, elevator, robot, numberOfJoints);
   }

//   @Ignore
   @Test
   public void floatingChainTest() throws UnreasonableAccelerationException
   {
      Random random = new Random(12651L);

      ArrayList<RevoluteJoint> joints = new ArrayList<>();
      int numberOfJoints = 6; 
      Vector3d[] jointAxes = new Vector3d[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointAxes[i] = RandomTools.generateRandomVector(random, 1.0);

      ScrewTestTools.RandomFloatingChain idRobot = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      RigidBody elevator = idRobot.getElevator();
      joints.addAll(idRobot.getRevoluteJoints());

      SCSRobotFromInverseDynamicsRobotModel robot = new SCSRobotFromInverseDynamicsRobotModel("robot", idRobot.getRootJoint());

      assertADotV(random, joints, elevator, robot, numberOfJoints + 1);
   }

   private void assertADotV(Random random, ArrayList<RevoluteJoint> joints, RigidBody elevator, SCSRobotFromInverseDynamicsRobotModel robot,int numJoints)
         throws UnreasonableAccelerationException
   {
      int numberOfDoFs = ScrewTools.computeDegreesOfFreedom(ScrewTools.computeSubtreeJoints(elevator));
      DenseMatrix64F v = new DenseMatrix64F(numberOfDoFs, 1);

      InverseDynamicsJoint[] idJoints = new InverseDynamicsJoint[numJoints]; 
      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);

      CentroidalMomentumMatrix centroidalMomentumMatrixCalculator = new CentroidalMomentumMatrix(elevator, centerOfMassFrame);

      a.reshape(6, numberOfDoFs);
      aPrevVal.reshape(6, numberOfDoFs);
      aDot.reshape(6, numberOfDoFs);

      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
      CentroidalMomentumRateADotVTerm aDotVAnalyticalCalculator = new CentroidalMomentumRateADotVTerm(elevator, centerOfMassFrame,
            centroidalMomentumMatrixCalculator, totalMass, v);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         ScrewTestTools.setRandomVelocities(joints, random);
         ScrewTestTools.setRandomPositions(joints, random);
         ScrewTestTools.setRandomTorques(joints, random);

         robot.updateJointPositions_ID_to_SCS();
         robot.updateJointVelocities_ID_to_SCS();
         robot.updateJointTorques_ID_to_SCS();

         centerOfMassFrame.update();

         centroidalMomentumMatrixCalculator.compute();
         aPrevVal.set(centroidalMomentumMatrixCalculator.getMatrix());

         robot.doDynamicsAndIntegrate(controlDT);
         robot.updateVelocities();
         robot.updateJointPositions_SCS_to_ID();
         robot.updateJointVelocities_SCS_to_ID();
         elevator.updateFramesRecursively();
         centerOfMassFrame.update();
         
         robot.packIdJoints(idJoints);
         ScrewTools.packJointVelocitiesMatrix(idJoints, v);

         // Compute aDotV analytically
         aDotVAnalyticalCalculator.compute();
         aDotVAnalytical.set(aDotVAnalyticalCalculator.getMatrix());

         // Compute aDotV numerically
         centroidalMomentumMatrixCalculator.compute();
         a.set(centroidalMomentumMatrixCalculator.getMatrix());
         MatrixTools.numericallyDifferentiate(aDot, aPrevVal, a, controlDT);
         CommonOps.mult(aDot, v, aDotVNumerical);

         smartPrintOutADotV(EPSILON);

         JUnitTools.assertMatrixEquals(aDotVNumerical, aDotVAnalytical, EPSILON);
      }
   }

   private void smartPrintOutADotV(double epsilon)
   {
      DenseMatrix64F difference = new DenseMatrix64F(aDotVNumerical.numRows, aDotVNumerical.numCols);
      CommonOps.subtract(aDotVNumerical, aDotVAnalytical, difference);

      for (int i = 0; i < difference.numRows; i++)
         if (Math.abs(difference.get(i, 0)) > epsilon)
            printOutADotV();
   }

   private void printOutADotV()
   {
      int numChar = 6;
      int precision = 3;
      String format = "%" + numChar + "." + precision + "f ";

      System.out.println("----------- ADotV -----------");
      System.out.println("Numerical: ||\tAnalytical:");
      for (int i = 0; i < aDotVNumerical.numRows; i++)
         System.out.printf(format + "\t   ||\t" + format + "\n", aDotVNumerical.get(i, 0), aDotVAnalytical.get(i, 0));
      System.out.println();
   }
}
