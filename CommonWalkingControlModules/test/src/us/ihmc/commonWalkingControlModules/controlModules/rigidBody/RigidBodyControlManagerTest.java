package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.junit.Test;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.SimpleContactPointPlaneBody;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationSettings;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class RigidBodyControlManagerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final Random random = new Random(94391892L);
   private static final double epsilon = 1.0E-10;

   private YoVariableRegistry testRegistry;
   private YoDouble yoTime;
   private RigidBody bodyToControl;

   private OneDoFJoint joint1;
   private OneDoFJoint joint2;

   private double q1_init = random.nextDouble();
   private double q2_init = random.nextDouble();

   private double q1_home = random.nextDouble();
   private double q2_home = random.nextDouble();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test
   public void testConstuctor()
   {
      createManager();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(expected = RuntimeException.class)
   public void testFailWithoutGains()
   {
      RigidBodyControlManager manager = createManager();
      manager.initialize();
      manager.compute();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test
   public void testInitialize()
   {
      // create manager
      RigidBodyControlManager manager = createManager();
      assertEquals(q2_init, joint2.getQ(), epsilon);
      assertEquals(q1_init, joint1.getQ(), epsilon);

      // compute
      setGainsAndWeights(manager);
      manager.initialize();
      manager.compute();
      assertEquals(RigidBodyControlMode.JOINTSPACE, manager.getActiveControlMode());

      // get commands and make sure they are initialized correctly
      {
         FeedbackControlCommand<?> feedbackControlCommand = manager.getFeedbackControlCommand();
         assertEquals(ControllerCoreCommandType.JOINTSPACE, feedbackControlCommand.getCommandType());
         JointspaceFeedbackControlCommand jointCommand = (JointspaceFeedbackControlCommand) feedbackControlCommand;
         assertEquals(1, jointCommand.getNumberOfJoints());
         assertEquals(jointCommand.getDesiredPosition(0), q2_init, epsilon);
      }

      // go to time when the joints should be at the home position
      yoTime.set(RigidBodyControlManager.INITIAL_GO_HOME_TIME);
      manager.compute();
      {
         FeedbackControlCommand<?> feedbackControlCommand = manager.getFeedbackControlCommand();
         assertEquals(ControllerCoreCommandType.JOINTSPACE, feedbackControlCommand.getCommandType());
         JointspaceFeedbackControlCommand jointCommand = (JointspaceFeedbackControlCommand) feedbackControlCommand;
         assertEquals(1, jointCommand.getNumberOfJoints());
         assertEquals(jointCommand.getDesiredPosition(0), q2_home, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test
   public void testTaskspaceMessage()
   {
      RigidBodyControlManager manager = createManager();
      setGainsAndWeights(manager);
      manager.initialize();
      manager.compute();

      double trajectoryTime = 1.0;
      Point3D position = EuclidCoreRandomTools.generateRandomPoint3D(random);
      Quaternion orientation = EuclidCoreRandomTools.generateRandomQuaternion(random);
      Vector3D linearVelocity = EuclidCoreRandomTools.generateRandomVector3D(random);
      Vector3D angularVelocity = EuclidCoreRandomTools.generateRandomVector3D(random);

      SE3Message message = new SE3Message(1, worldFrame);
      message.setTrajectoryPoint(0, trajectoryTime, position, orientation, linearVelocity, angularVelocity, worldFrame);

      SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();
      boolean angularXSelected = random.nextBoolean();
      boolean angularYSelected = random.nextBoolean();
      boolean angularZSelected = random.nextBoolean();
      selectionMatrix6D.setAngularAxisSelection(angularXSelected, angularYSelected, angularZSelected);

      boolean linearXSelected = random.nextBoolean();
      boolean linearYSelected = random.nextBoolean();
      boolean linearZSelected = random.nextBoolean();
      selectionMatrix6D.setLinearAxisSelection(linearXSelected, linearYSelected, linearZSelected);
      message.setSelectionMatrix(selectionMatrix6D);

      WeightMatrix6D weightMatrix = new WeightMatrix6D();
      double angularXWeight = random.nextDouble();
      double angularYWeight = random.nextDouble();
      double angularZWeight = random.nextDouble();
      weightMatrix.setAngularWeights(angularXWeight, angularYWeight, angularZWeight);

      double linearXWeight = random.nextDouble();
      double linearYWeight = random.nextDouble();
      double linearZWeight = random.nextDouble();
      weightMatrix.setLinearWeights(linearXWeight, linearYWeight, linearZWeight);
      message.setWeightMatrix(weightMatrix);

      SE3Command command = new SE3Command();
      command.set(worldFrame, worldFrame, message);
      manager.handleTaskspaceTrajectoryCommand(command);
      manager.compute();

      assertEquals(RigidBodyControlMode.TASKSPACE, manager.getActiveControlMode());

      FramePoint desiredPosition = new FramePoint();
      FrameVector desiredLinearVelocity = new FrameVector();
      FrameVector feedForwardLinearAcceleration = new FrameVector();
      FrameOrientation desiredOrientation = new FrameOrientation();
      FrameVector desiredAngularVelocity = new FrameVector();
      FrameVector feedForwardAngularAcceleration = new FrameVector();

      ReferenceFrame bodyFrame = bodyToControl.getBodyFixedFrame();
      FramePoint initialPosition = new FramePoint(bodyFrame);
      FrameOrientation initialOrientation = new FrameOrientation(bodyFrame);
      initialPosition.changeFrame(worldFrame);
      initialOrientation.changeFrame(worldFrame);

      // get commands and make sure they are initialized correctly
      {
         FeedbackControlCommand<?> feedbackControlCommand = manager.getFeedbackControlCommand();
         assertEquals(ControllerCoreCommandType.TASKSPACE, feedbackControlCommand.getCommandType());
         SpatialFeedbackControlCommand taskspaceCommand = (SpatialFeedbackControlCommand) feedbackControlCommand;

         assertEquals(taskspaceCommand.getEndEffector().getNameBasedHashCode(), bodyToControl.getNameBasedHashCode());
         taskspaceCommand.getIncludingFrame(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
         taskspaceCommand.getIncludingFrame(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

         initialPosition.checkReferenceFrameMatch(desiredPosition);
         EuclidCoreTestTools.assertTuple3DEquals(initialPosition.getPoint(), desiredPosition.getPoint(), epsilon);
         initialOrientation.checkReferenceFrameMatch(desiredOrientation);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(initialOrientation.getQuaternion(), desiredOrientation.getQuaternion(), epsilon);
      }

      // go forward to the end of the trajectory
      yoTime.set(trajectoryTime);
      manager.compute();
      {
         FeedbackControlCommand<?> feedbackControlCommand = manager.getFeedbackControlCommand();
         assertEquals(ControllerCoreCommandType.TASKSPACE, feedbackControlCommand.getCommandType());
         SpatialFeedbackControlCommand taskspaceCommand = (SpatialFeedbackControlCommand) feedbackControlCommand;

         assertEquals(taskspaceCommand.getEndEffector().getNameBasedHashCode(), bodyToControl.getNameBasedHashCode());
         taskspaceCommand.getIncludingFrame(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
         taskspaceCommand.getIncludingFrame(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

         initialPosition.checkReferenceFrameMatch(desiredPosition);
         EuclidCoreTestTools.assertTuple3DEquals(position, desiredPosition.getPoint(), epsilon);
         initialOrientation.checkReferenceFrameMatch(desiredOrientation);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(orientation, desiredOrientation.getQuaternion(), epsilon);

         EuclidCoreTestTools.assertTuple3DEquals(linearVelocity, desiredLinearVelocity.getVector(), epsilon);
         EuclidCoreTestTools.assertTuple3DEquals(angularVelocity, desiredAngularVelocity.getVector(), epsilon);

         SpatialAccelerationCommand spatialAccelerationCommand = taskspaceCommand.getSpatialAccelerationCommand();

         //This looks redundant but this is testing the equals is correct here,
         SelectionMatrix6D selectionMatrixFromCommand = new SelectionMatrix6D();
         spatialAccelerationCommand.getSelectionMatrix(selectionMatrixFromCommand);
         assertTrue(selectionMatrix6D.equals(selectionMatrixFromCommand));

         assertNull(selectionMatrix6D.getAngularSelectionFrame());
         assertNull(selectionMatrix6D.getLinearSelectionFrame());
         SelectionMatrix3D selectionMatrixLinearPart = selectionMatrix6D.getLinearPart();
         assertEquals(linearXSelected, selectionMatrixLinearPart.isXSelected());
         assertEquals(linearYSelected, selectionMatrixLinearPart.isYSelected());
         assertEquals(linearZSelected, selectionMatrixLinearPart.isZSelected());

         SelectionMatrix3D selectionMatrixAngularPart = selectionMatrix6D.getAngularPart();
         assertEquals(angularXSelected, selectionMatrixAngularPart.isXSelected());
         assertEquals(angularYSelected, selectionMatrixAngularPart.isYSelected());
         assertEquals(angularZSelected, selectionMatrixAngularPart.isZSelected());

         WeightMatrix6D weightMatrixFromCommand = spatialAccelerationCommand.getWeightMatrix();

         //This looks redundant but this is testing the equals is correct here,
         WeightMatrix3D weightMartrixFromCommandLinearPart = weightMatrixFromCommand.getLinearPart();
         assertEquals(linearXWeight, weightMartrixFromCommandLinearPart.getXAxisWeight(), 1e-08);
         assertEquals(linearYWeight, weightMartrixFromCommandLinearPart.getYAxisWeight(), 1e-08);
         assertEquals(linearZWeight, weightMartrixFromCommandLinearPart.getZAxisWeight(), 1e-08);

         WeightMatrix3D weightMartrixFromCommandAngularPart = weightMatrixFromCommand.getAngularPart();
         assertEquals(angularXWeight, weightMartrixFromCommandAngularPart.getXAxisWeight(), 1e-08);
         assertEquals(angularYWeight, weightMartrixFromCommandAngularPart.getYAxisWeight(), 1e-08);
         assertEquals(angularZWeight, weightMartrixFromCommandAngularPart.getZAxisWeight(), 1e-08);

         assertTrue(weightMatrix.equals(weightMatrixFromCommand));
         assertNull(weightMatrix.getAngularWeightFrame());
         assertNull(weightMatrix.getLinearWeightFrame());
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test
   public void testTaskspaceWeightAndSelectionMatrixFromMessage()
   {
      RigidBodyControlManager manager = createManager();
      setGainsAndWeights(manager);
      manager.initialize();
      manager.compute();

      double trajectoryTime = 1.0;

      RigidBodyTransform randomTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

      List<ReferenceFrame> referenceFrames = new ArrayList<>();
      referenceFrames.add(null);
      referenceFrames.add(ReferenceFrame.constructFrameWithUnchangingTransformToParent("blop1Bis", ReferenceFrame.getWorldFrame(), randomTransform));
      referenceFrames.add(ReferenceFrame.getWorldFrame());
      referenceFrames.add(ReferenceFrame.constructFrameWithUnchangingTransformToParent("blop1", ReferenceFrame.getWorldFrame(), randomTransform));
      referenceFrames.add(ReferenceFrame.generateRandomReferenceFrame("blop2", random, ReferenceFrame.getWorldFrame()));

      ReferenceFrameHashCodeResolver resolver = new ReferenceFrameHashCodeResolver(referenceFrames);
      for (int i = 0; i < 50; i++)
      {
         Point3D position = EuclidCoreRandomTools.generateRandomPoint3D(random);
         Quaternion orientation = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D linearVelocity = EuclidCoreRandomTools.generateRandomVector3D(random);
         Vector3D angularVelocity = EuclidCoreRandomTools.generateRandomVector3D(random);

         SE3Message message = new SE3Message(1, worldFrame);
         message.setTrajectoryPoint(0, trajectoryTime, position, orientation, linearVelocity, angularVelocity, worldFrame);
         message.setExecutionMode(ExecutionMode.OVERRIDE, -1);

         SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();
         boolean angularXSelected = random.nextBoolean();
         boolean angularYSelected = random.nextBoolean();
         boolean angularZSelected = random.nextBoolean();
         selectionMatrix6D.setAngularAxisSelection(angularXSelected, angularYSelected, angularZSelected);

         boolean linearXSelected = random.nextBoolean();
         boolean linearYSelected = random.nextBoolean();
         boolean linearZSelected = random.nextBoolean();
         selectionMatrix6D.setLinearAxisSelection(linearXSelected, linearYSelected, linearZSelected);

         ReferenceFrame angularSelectionFrame = referenceFrames.get(random.nextInt(referenceFrames.size()));
         ReferenceFrame linearSelectionFrame = referenceFrames.get(random.nextInt(referenceFrames.size()));
         selectionMatrix6D.setSelectionFrames(angularSelectionFrame, linearSelectionFrame);
         message.setSelectionMatrix(selectionMatrix6D);

         WeightMatrix6D weightMatrix = new WeightMatrix6D();
         double angularXWeight = random.nextDouble();
         double angularYWeight = random.nextDouble();
         double angularZWeight = random.nextDouble();
         weightMatrix.setAngularWeights(angularXWeight, angularYWeight, angularZWeight);

         double linearXWeight = random.nextDouble();
         double linearYWeight = random.nextDouble();
         double linearZWeight = random.nextDouble();
         weightMatrix.setLinearWeights(linearXWeight, linearYWeight, linearZWeight);

         ReferenceFrame angularWeightFrame = referenceFrames.get(random.nextInt(referenceFrames.size()));
         ReferenceFrame linearWeightFrame = referenceFrames.get(random.nextInt(referenceFrames.size()));
         weightMatrix.setWeightFrames(angularWeightFrame, linearWeightFrame);
         message.setWeightMatrix(weightMatrix);

         SE3Command command = new SE3Command();
         command.set(resolver, message);
         manager.handleTaskspaceTrajectoryCommand(command);
         manager.compute();

         assertEquals(RigidBodyControlMode.TASKSPACE, manager.getActiveControlMode());

         ReferenceFrame bodyFrame = bodyToControl.getBodyFixedFrame();
         FramePoint initialPosition = new FramePoint(bodyFrame);
         FrameOrientation initialOrientation = new FrameOrientation(bodyFrame);
         initialPosition.changeFrame(worldFrame);
         initialOrientation.changeFrame(worldFrame);

         FeedbackControlCommand<?> feedbackControlCommand = manager.getFeedbackControlCommand();
         assertEquals(ControllerCoreCommandType.TASKSPACE, feedbackControlCommand.getCommandType());
         SpatialFeedbackControlCommand taskspaceCommand = (SpatialFeedbackControlCommand) feedbackControlCommand;

         SpatialAccelerationCommand spatialAccelerationCommand = taskspaceCommand.getSpatialAccelerationCommand();

         //This looks redundant but this is testing the equals is correct here,
         SelectionMatrix6D selectionMatrixFromCommand = new SelectionMatrix6D();
         spatialAccelerationCommand.getSelectionMatrix(selectionMatrixFromCommand);

         SelectionMatrix3D selectionMatrixLinearPart = selectionMatrix6D.getLinearPart();
         assertEquals(linearXSelected, selectionMatrixLinearPart.isXSelected());
         assertEquals(linearYSelected, selectionMatrixLinearPart.isYSelected());
         assertEquals(linearZSelected, selectionMatrixLinearPart.isZSelected());

         SelectionMatrix3D selectionMatrixAngularPart = selectionMatrix6D.getAngularPart();
         assertEquals(angularXSelected, selectionMatrixAngularPart.isXSelected());
         assertEquals(angularYSelected, selectionMatrixAngularPart.isYSelected());
         assertEquals(angularZSelected, selectionMatrixAngularPart.isZSelected());

         assertEquals(angularSelectionFrame, selectionMatrix6D.getAngularSelectionFrame());
         assertEquals(linearSelectionFrame, selectionMatrix6D.getLinearSelectionFrame());
         assertTrue(selectionMatrix6D.equals(selectionMatrixFromCommand));

         WeightMatrix6D weightMatrixFromCommand = spatialAccelerationCommand.getWeightMatrix();

         //This looks redundant but this is testing the equals is correct here,
         WeightMatrix3D weightMartrixFromCommandLinearPart = weightMatrixFromCommand.getLinearPart();
         assertEquals(linearXWeight, weightMartrixFromCommandLinearPart.getXAxisWeight(), 1e-08);
         assertEquals(linearYWeight, weightMartrixFromCommandLinearPart.getYAxisWeight(), 1e-08);
         assertEquals(linearZWeight, weightMartrixFromCommandLinearPart.getZAxisWeight(), 1e-08);

         WeightMatrix3D weightMartrixFromCommandAngularPart = weightMatrixFromCommand.getAngularPart();
         assertEquals(angularXWeight, weightMartrixFromCommandAngularPart.getXAxisWeight(), 1e-08);
         assertEquals(angularYWeight, weightMartrixFromCommandAngularPart.getYAxisWeight(), 1e-08);
         assertEquals(angularZWeight, weightMartrixFromCommandAngularPart.getZAxisWeight(), 1e-08);

         assertEquals(angularWeightFrame, weightMatrix.getAngularWeightFrame());
         assertEquals(linearWeightFrame, weightMatrix.getLinearWeightFrame());
         assertTrue(weightMatrix.equals(weightMatrixFromCommand));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test
   public void testTaskspaceMessageWithCustomControlFrame()
   {
      RigidBodyControlManager manager = createManager();
      setGainsAndWeights(manager);
      manager.initialize();
      manager.compute();

      double trajectoryTime = 1.0;
      Point3D position = EuclidCoreRandomTools.generateRandomPoint3D(random);
      Quaternion orientation = EuclidCoreRandomTools.generateRandomQuaternion(random);
      Vector3D linearVelocity = EuclidCoreRandomTools.generateRandomVector3D(random);
      Vector3D angularVelocity = EuclidCoreRandomTools.generateRandomVector3D(random);

      Point3D controlFramePosition = EuclidCoreRandomTools.generateRandomPoint3D(random);
      Quaternion controlFrameOrientation = EuclidCoreRandomTools.generateRandomQuaternion(random);

      SE3Message message = new SE3Message(1, worldFrame);
      message.setControlFramePosition(controlFramePosition);
      message.setControlFrameOrientation(controlFrameOrientation);
      message.setUseCustomControlFrame(true);
      message.setTrajectoryPoint(0, trajectoryTime, position, orientation, linearVelocity, angularVelocity, worldFrame);

      SE3Command command = new SE3Command();
      command.set(worldFrame, worldFrame, message);
      manager.handleTaskspaceTrajectoryCommand(command);
      manager.compute();

      assertEquals(RigidBodyControlMode.TASKSPACE, manager.getActiveControlMode());

      FramePoint desiredPosition = new FramePoint();
      FrameVector desiredLinearVelocity = new FrameVector();
      FrameVector feedForwardLinearAcceleration = new FrameVector();
      FrameOrientation desiredOrientation = new FrameOrientation();
      FrameVector desiredAngularVelocity = new FrameVector();
      FrameVector feedForwardAngularAcceleration = new FrameVector();
      FrameOrientation actualControlFrameOrientation = new FrameOrientation();
      FramePoint actualControlFramePosition = new FramePoint();

      ReferenceFrame bodyFrame = bodyToControl.getBodyFixedFrame();
      PoseReferenceFrame controlFrame = new PoseReferenceFrame("TestControlFrame", bodyFrame);
      controlFrame.setPoseAndUpdate(controlFramePosition, controlFrameOrientation);
      FramePoint initialPosition = new FramePoint(controlFrame);
      FrameOrientation initialOrientation = new FrameOrientation(controlFrame);
      initialPosition.changeFrame(worldFrame);
      initialOrientation.changeFrame(worldFrame);

      // get commands and make sure they are initialized correctly
      {
         FeedbackControlCommand<?> feedbackControlCommand = manager.getFeedbackControlCommand();
         assertEquals(ControllerCoreCommandType.TASKSPACE, feedbackControlCommand.getCommandType());
         SpatialFeedbackControlCommand taskspaceCommand = (SpatialFeedbackControlCommand) feedbackControlCommand;

         assertEquals(taskspaceCommand.getEndEffector().getNameBasedHashCode(), bodyToControl.getNameBasedHashCode());
         taskspaceCommand.getIncludingFrame(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
         taskspaceCommand.getIncludingFrame(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

         initialPosition.checkReferenceFrameMatch(desiredPosition);
         EuclidCoreTestTools.assertTuple3DEquals(initialPosition.getPoint(), desiredPosition.getPoint(), epsilon);
         initialOrientation.checkReferenceFrameMatch(desiredOrientation);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(initialOrientation.getQuaternion(), desiredOrientation.getQuaternion(), 1e-10);

         taskspaceCommand.getControlFramePoseIncludingFrame(actualControlFramePosition, actualControlFrameOrientation);
         actualControlFramePosition.checkReferenceFrameMatch(bodyFrame);
         actualControlFrameOrientation.checkReferenceFrameMatch(bodyFrame);
         EuclidCoreTestTools.assertTuple3DEquals(controlFramePosition, actualControlFramePosition.getPoint(), epsilon);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(controlFrameOrientation, actualControlFrameOrientation.getQuaternion(), epsilon);
      }

      // go forward to the end of the trajectory
      yoTime.set(trajectoryTime);
      manager.compute();
      {
         FeedbackControlCommand<?> feedbackControlCommand = manager.getFeedbackControlCommand();
         assertEquals(ControllerCoreCommandType.TASKSPACE, feedbackControlCommand.getCommandType());
         SpatialFeedbackControlCommand taskspaceCommand = (SpatialFeedbackControlCommand) feedbackControlCommand;

         assertEquals(taskspaceCommand.getEndEffector().getNameBasedHashCode(), bodyToControl.getNameBasedHashCode());
         taskspaceCommand.getIncludingFrame(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
         taskspaceCommand.getIncludingFrame(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

         initialPosition.checkReferenceFrameMatch(desiredPosition);
         EuclidCoreTestTools.assertTuple3DEquals(position, desiredPosition.getPoint(), epsilon);
         initialOrientation.checkReferenceFrameMatch(desiredOrientation);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(orientation, desiredOrientation.getQuaternion(), epsilon);

         EuclidCoreTestTools.assertTuple3DEquals(linearVelocity, desiredLinearVelocity.getVector(), epsilon);
         EuclidCoreTestTools.assertTuple3DEquals(angularVelocity, desiredAngularVelocity.getVector(), epsilon);
      }
   }

   private RigidBodyControlManager createManager()
   {
      // create test registry
      testRegistry = new YoVariableRegistry(getClass().getSimpleName());

      // create time variable
      yoTime = new YoDouble("yoTime", testRegistry);

      // create a dummy robot with elevator, two joints, and two rigid bodies
      RigidBody elevator = new RigidBody("elevator", worldFrame);
      joint1 = ScrewTools.addRevoluteJoint("Joint1", elevator, new Vector3D(), new Vector3D(1.0, 0.0, 0.0));
      RigidBody link1 = ScrewTools.addRigidBody("Link1", joint1, new Matrix3D(), 0.0, new Vector3D());
      joint2 = ScrewTools.addRevoluteJoint("Joint2", link1, new Vector3D(), new Vector3D(1.0, 0.0, 0.0));
      RigidBody link2 = ScrewTools.addRigidBody("Link2", joint2, new Matrix3D(), 0.0, new Vector3D());

      joint1.setQ(q1_init);
      joint2.setQ(q2_init);

      // setup a home configuration for the robot
      TObjectDoubleHashMap<String> homeConfiguration = new TObjectDoubleHashMap<>();
      homeConfiguration.put(joint1.getName(), q1_home);
      homeConfiguration.put(joint2.getName(), q2_home);

      // no position controlled joints
      List<String> positionControlledJointNames = new ArrayList<>();
      Map<String, JointAccelerationIntegrationSettings> integrationSettings = new HashMap<>();

      // add some possible trajectory frames
      Collection<ReferenceFrame> trajectoryFrames = new ArrayList<>();
      trajectoryFrames.add(link1.getBodyFixedFrame());
      trajectoryFrames.add(link2.getBodyFixedFrame());

      // create a contactable body
      ContactablePlaneBody contactableBody = new SimpleContactPointPlaneBody("ContactableBody", link2, new RigidBodyTransform());

      // use default control and base frames
      bodyToControl = link2;
      RigidBody baseBody = link1;
      ReferenceFrame controlFrame = bodyToControl.getBodyFixedFrame();
      ReferenceFrame baseFrame = baseBody.getBodyFixedFrame();

      return new RigidBodyControlManager(bodyToControl, baseBody, elevator, homeConfiguration, null, positionControlledJointNames, integrationSettings,
            trajectoryFrames, controlFrame, baseFrame, contactableBody, yoTime, null, testRegistry);
   }

   private void setGainsAndWeights(RigidBodyControlManager manager)
   {
      // setup gains and weights to be all zero with weights 1.0
      Map<String, YoPIDGains> jointspaceGains = new HashMap<>();
      jointspaceGains.put(joint1.getName(), new YoPIDGains("Joint1Gains", testRegistry));
      jointspaceGains.put(joint2.getName(), new YoPIDGains("Joint2Gains", testRegistry));
      YoOrientationPIDGainsInterface taskspaceOrientationGains = new YoSymmetricSE3PIDGains("OrientationGains", testRegistry);
      YoPositionPIDGainsInterface taskspacePositionGains = new YoSymmetricSE3PIDGains("PositionGains", testRegistry);
      TObjectDoubleHashMap<String> jointspaceWeights = new TObjectDoubleHashMap<>();
      jointspaceWeights.put(joint1.getName(), 1.0);
      jointspaceWeights.put(joint2.getName(), 1.0);
      Vector3D taskspaceAngularWeight = new Vector3D(1.0, 1.0, 1.0);
      Vector3D taskspaceLinearWeight = new Vector3D(1.0, 1.0, 1.0);
      TObjectDoubleHashMap<String> userModeWeights = new TObjectDoubleHashMap<>();
      userModeWeights.put(joint1.getName(), 1.0);
      userModeWeights.put(joint2.getName(), 1.0);

      manager.setGains(jointspaceGains, taskspaceOrientationGains, taskspacePositionGains);
      manager.setWeights(jointspaceWeights, taskspaceAngularWeight, taskspaceLinearWeight, userModeWeights);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(RigidBodyControlManager.class, RigidBodyControlManagerTest.class);
   }
}