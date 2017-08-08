package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import static us.ihmc.robotics.weightMatrices.SolverWeightLevels.*;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;

/**
 * {@link MomentumRateCommand} is a command meant to be submitted to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link MomentumRateCommand} is to notify the inverse dynamics optimization
 * module to get the robot to achieve the desired rate of change of momentum during the next control
 * tick.
 * </p>
 * <p>
 * This command can notably be used to control the center of mass acceleration by using the linear
 * part of the rate of change of momentum.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class MomentumRateCommand implements InverseDynamicsCommand<MomentumRateCommand>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   /**
    * It defines the 6-components of the desired rate of change of momentum to achieve for the next
    * control.
    * <p>
    * More precisely, it represents the desired rate of change of momentum at the center of mass
    * while the data is expressed in world frame.
    * </p>
    */
   private final DenseMatrix64F momentumRateOfChange = new DenseMatrix64F(Momentum.SIZE, 1);
   /**
    * Weights on a per component basis to use in the optimization function. A higher weight means
    * higher priority of this task.
    */
   private final DenseMatrix64F weightVector = new DenseMatrix64F(Momentum.SIZE, 1);
   /**
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) for the rate of change
    * of momentum that are to be controlled. It is initialized such that the controller will by
    * default control all the DoFs.
    * <p>
    * If the selection frame is not set, it is assumed that the selection frame is equal to the
    * centroidal frame.
    * </p>
    */
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public MomentumRateCommand()
   {
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(MomentumRateCommand other)
   {
      weightVector.set(other.weightVector);
      selectionMatrix.set(other.selectionMatrix);
      momentumRateOfChange.set(other.momentumRateOfChange);
   }

   /**
    * Sets the desired momentum rate to submit for the optimization to zero.
    */
   public void setMomentumRateToZero()
   {
      momentumRateOfChange.zero();
   }

   /**
    * Sets the desired rate of change of momentum to submit for the optimization.
    * <p>
    * It is assumed to represent the desired rate of change of momentum at the center of mass while
    * the data is expressed in world frame.
    * </p>
    * 
    * @param momentumRateOfChange the desired momentum rate at the center of mass expressed in world
    *           frame. Not modified.
    */
   public void setMomentumRate(DenseMatrix64F momentumRateOfChange)
   {
      this.momentumRateOfChange.set(momentumRateOfChange);
   }

   /**
    * Sets the desired rate of change of momentum to submit for the optimization.
    * <p>
    * It is assumed to represent the desired rate of change of momentum at the center of mass while
    * the data is expressed in world frame.
    * </p>
    * 
    * @param angularMomentumRateOfChange the desired angular momentum rate at the center of mass
    *           expressed in world frame. Not modified.
    * @param linearMomentumRateOfChange the desired linear momentum rate in world frame. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code angularMomentumRateOfChange} or
    *            {@code linearMomentumRateOfChange} is not expressed in world frame.
    */
   public void setMomentumRate(FrameVector angularMomentumRateOfChange, FrameVector linearMomentumRateOfChange)
   {
      angularMomentumRateOfChange.checkReferenceFrameMatch(worldFrame);
      linearMomentumRateOfChange.checkReferenceFrameMatch(worldFrame);

      angularMomentumRateOfChange.get(0, momentumRateOfChange);
      linearMomentumRateOfChange.get(3, momentumRateOfChange);
   }

   /**
    * Sets the desired rate of change of angular momentum to submit for the optimization and sets
    * the linear part to zero.
    * <p>
    * It is assumed to represent the desired rate of change of momentum at the center of mass while
    * the data is expressed in world frame.
    * </p>
    * 
    * @param angularMomentumRateOfChange the desired angular momentum rate at the center of mass
    *           expressed in world frame. Not modified.
    * @throws ReferenceFrameMismatchException if {@code angularMomentumRateOfChange} is not
    *            expressed in world frame.
    */
   public void setAngularMomentumRate(FrameVector angularMomentumRateOfChange)
   {
      angularMomentumRateOfChange.checkReferenceFrameMatch(worldFrame);

      momentumRateOfChange.zero();
      angularMomentumRateOfChange.get(0, momentumRateOfChange);
   }

   /**
    * Sets the desired rate of change of linear momentum to submit for the optimization and sets the
    * angular part to zero.
    * 
    * @param linearMomentumRateOfChange the desired linear momentum rate in world frame. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code linearMomentumRateOfChange} is not expressed
    *            in world frame.
    */
   public void setLinearMomentumRate(FrameVector linearMomentumRateOfChange)
   {
      linearMomentumRateOfChange.checkReferenceFrameMatch(worldFrame);

      momentumRateOfChange.zero();
      linearMomentumRateOfChange.get(3, momentumRateOfChange);
   }

   /**
    * Sets the desired rate of change of linear momentum in the XY-plane to submit for the
    * optimization and sets the angular part and the z component of the linear part to zero.
    * 
    * @param linearMomentumRateOfChange the desired linear momentum rate in world frame. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code linearMomentumRateOfChange} is not expressed
    *            in world frame.
    */
   public void setLinearMomentumXYRate(FrameVector2d linearMomentumRateOfChange)
   {
      linearMomentumRateOfChange.checkReferenceFrameMatch(worldFrame);

      momentumRateOfChange.zero();
      linearMomentumRateOfChange.get(3, momentumRateOfChange);
   }

   /**
    * Sets the selection matrix to be used for the next control tick to the 6-by-6 identity matrix.
    * <p>
    * This specifies that the 6 degrees of freedom for the rate of change of momentum are to be
    * controlled.
    * </p>
    */
   public void setSelectionMatrixToIdentity()
   {
      selectionMatrix.resetSelection();
   }

   /**
    * Convenience method that sets up the selection matrix such that only the linear part of this
    * command will be considered in the optimization.
    */
   public void setSelectionMatrixForLinearControl()
   {
      selectionMatrix.setToLinearSelectionOnly();
   }

   /**
    * Convenience method that sets up the selection matrix by disabling the angular part of this
    * command and applying the given selection matrix to the linear part.
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the
    * selection frame is equal to the control frame.
    * </p>
    * 
    * @param linearSelectionMatrix the selection matrix to apply to the linear part of this command.
    *           Not modified.
    */
   public void setSelectionMatrixForLinearControl(SelectionMatrix3D linearSelectionMatrix)
   {
      selectionMatrix.clearSelection();
      selectionMatrix.setLinearPart(linearSelectionMatrix);
   }

   /**
    * Convenience method that sets up the selection matrix such that only the x and y components of
    * the linear part of this command will be considered in the optimization.
    */
   public void setSelectionMatrixForLinearXYControl()
   {
      selectionMatrix.setToLinearSelectionOnly();
      selectionMatrix.selectLinearZ(false);
   }

   /**
    * Convenience method that sets up the selection matrix such that only the angular part of this
    * command will be considered in the optimization.
    */
   public void setSelectionMatrixForAngularControl()
   {
      selectionMatrix.setToAngularSelectionOnly();
   }

   /**
    * Sets this command's selection matrix to the given one.
    * <p>
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) for the rate of change
    * momentum that are to be controlled. It is initialized such that the controller will by default
    * control all the DoFs.
    * </p>
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the
    * selection frame is equal to the centroidal frame.
    * </p>
    * 
    * @param selectionMatrix the selection matrix to copy data from. Not modified.
    */
   public void setSelectionMatrix(SelectionMatrix6D selectionMatrix)
   {
      this.selectionMatrix.set(selectionMatrix);
   }

   /**
    * Sets all the weights to {@link SolverWeightLevels#HARD_CONSTRAINT} such that this command will
    * be treated as a hard constraint.
    * <p>
    * This is usually undesired as with improper commands setup as hard constraints the optimization
    * problem can simply be impossible to solve.
    * </p>
    */
   public void setAsHardConstraint()
   {
      setWeight(HARD_CONSTRAINT);
   }

   /**
    * Sets the weight to use in the optimization problem.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param weight the weight value to use for this command.
    */
   public void setWeight(double weight)
   {
      for (int i = 0; i < Momentum.SIZE; i++)
         weightVector.set(i, 0, weight);
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param angular the weight to use for the angular part of this command. Not modified.
    * @param linear the weight to use for the linear part of this command. Not modified.
    */
   public void setWeight(double angular, double linear)
   {
      for (int i = 0; i < 3; i++)
         weightVector.set(i, 0, angular);
      for (int i = 3; i < Momentum.SIZE; i++)
         weightVector.set(i, 0, linear);
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param angularX the weight to use for the x-axis of the angular part of this command.
    * @param angularY the weight to use for the y-axis of the angular part of this command.
    * @param angularZ the weight to use for the z-axis of the angular part of this command.
    * @param linearX the weight to use for the x-axis of the linear part of this command.
    * @param linearY the weight to use for the y-axis of the linear part of this command.
    * @param linearZ the weight to use for the z-axis of the linear part of this command.
    */
   public void setWeights(double angularX, double angularY, double angularZ, double linearX, double linearY, double linearZ)
   {
      weightVector.set(0, 0, angularX);
      weightVector.set(1, 0, angularY);
      weightVector.set(2, 0, angularZ);
      weightVector.set(3, 0, linearX);
      weightVector.set(4, 0, linearY);
      weightVector.set(5, 0, linearZ);
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param weight dense matrix holding the weights to use for each component of the desired rate
    *           of change of momentum. It is expected to be a 6-by-1 vector ordered as:
    *           {@code angularX}, {@code angularY}, {@code angularZ}, {@code linearX},
    *           {@code linearY}, {@code linearZ}. Not modified.
    */
   public void setWeights(DenseMatrix64F weight)
   {
      for (int i = 0; i < Momentum.SIZE; i++)
      {
         weightVector.set(i, 0, weight.get(i, 0));
      }
   }

   /**
    * Sets the weights to use in the optimization problem for each rotational degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param angular the weights to use for the angular part of this command. Not modified.
    */
   public void setAngularWeights(Tuple3DReadOnly angular)
   {
      weightVector.set(0, 0, angular.getX());
      weightVector.set(1, 0, angular.getY());
      weightVector.set(2, 0, angular.getZ());
   }

   /**
    * Sets the weights to use in the optimization problem for each translational degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param linear the weights to use for the angular part of this command. Not modified.
    */
   public void setLinearWeights(Tuple3DReadOnly linear)
   {
      weightVector.set(3, 0, linear.getX());
      weightVector.set(4, 0, linear.getY());
      weightVector.set(5, 0, linear.getZ());
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param angular the weights to use for the angular part of this command. Not modified.
    * @param linear the weights to use for the linear part of this command. Not modified.
    */
   public void setWeights(Tuple3DReadOnly angular, Tuple3DReadOnly linear)
   {
      weightVector.set(0, 0, angular.getX());
      weightVector.set(1, 0, angular.getY());
      weightVector.set(2, 0, angular.getZ());
      weightVector.set(3, 0, linear.getX());
      weightVector.set(4, 0, linear.getY());
      weightVector.set(5, 0, linear.getZ());
   }

   /**
    * Sets the weights to use in the optimization problem for each rotational degree of freedom to
    * zero.
    * <p>
    * By doing so, the angular part of this command will be ignored during the optimization. Note
    * that it is less expensive to call {@link #setSelectionMatrixForLinearControl()} as by doing so
    * the angular part will not be submitted to the optimization.
    * </p>
    */
   public void setAngularWeightsToZero()
   {
      for (int i = 0; i < 3; i++)
         weightVector.set(i, 0, 0.0);
   }

   /**
    * Sets the weights to use in the optimization problem for each translational degree of freedom
    * to zero.
    * <p>
    * By doing so, the linear part of this command will be ignored during the optimization. Note
    * that it is less expensive to call {@link #setSelectionMatrixForAngularControl()} as by doing
    * so the linear part will not be submitted to the optimization.
    * </p>
    */
   public void setLinearWeightsToZero()
   {
      for (int i = 3; i < Momentum.SIZE; i++)
         weightVector.set(i, 0, 0.0);
   }

   /**
    * Finds if this command is to be considered as a hard constraint during the optimization.
    * <p>
    * This command is considered to be a hard constraint if at least one of the weights is equal to
    * {@link SolverWeightLevels#HARD_CONSTRAINT}.
    * </p>
    * 
    * @return {@code true} if this command should be considered as a hard constraint, {@code false}
    *         is it should be part of the optimization objective.
    */
   public boolean isHardConstraint()
   {
      for (int i = 0; i < Momentum.SIZE; i++)
      {
         if (weightVector.get(i, 0) == HARD_CONSTRAINT)
            return true;
      }
      return false;
   }

   /**
    * Packs the weights to use for this command in {@code weightMatrixToPack} as follows:
    * 
    * <pre>
    *     / w0 0  0  0  0  0  \
    *     | 0  w1 0  0  0  0  |
    * W = | 0  0  w2 0  0  0  |
    *     | 0  0  0  w3 0  0  |
    *     | 0  0  0  0  w4 0  |
    *     \ 0  0  0  0  0  w5 /
    * </pre>
    * <p>
    * The three first weights (w0, w1, w2) are the weights to use for the angular part of this
    * command. The three others (w3, w4, w5) are for the linear part.
    * </p>
    * <p>
    * The packed matrix is a 6-by-6 matrix independently from the selection matrix.
    * </p>
    * 
    * @param weightMatrixToPack the weight matrix to use in the optimization. The given matrix is
    *           reshaped to ensure proper size. Modified.
    */
   public void getWeightMatrix(DenseMatrix64F weightMatrixToPack)
   {
      weightMatrixToPack.reshape(Momentum.SIZE, Momentum.SIZE);
      CommonOps.setIdentity(weightMatrixToPack);
      for (int i = 0; i < Momentum.SIZE; i++)
         weightMatrixToPack.set(i, i, weightVector.get(i, 0));
   }

   /**
    * Returns the reference to the 6-by-1 weight vector to use with this command:
    * 
    * <pre>
    *     / w0 \
    *     | w1 |
    * W = | w2 |
    *     | w3 |
    *     | w4 |
    *     \ w5 /
    * </pre>
    * <p>
    * The three first weights (w0, w1, w2) are the weights to use for the angular part of this
    * command. The three others (w3, w4, w5) are for the linear part.
    * </p>
    * 
    * @return the reference to the weights to use with this command.
    */
   public DenseMatrix64F getWeightVector()
   {
      return weightVector;
   }

   /**
    * Gets the 6-by-6 selection matrix expressed in the given {@code destinationFrame} to use with
    * this command.
    * 
    * @param destinationFrame the reference frame in which the selection matrix should be expressed
    *           in.
    * @param selectionMatrixToPack the dense-matrix in which the selection matrix of this command is
    *           stored in. Modified.
    */
   public void getSelectionMatrix(ReferenceFrame destinationFrame, DenseMatrix64F selectionMatrixToPack)
   {
      selectionMatrix.getCompactSelectionMatrixInFrame(destinationFrame, selectionMatrixToPack);
   }

   /**
    * Packs the value of the selection matrix carried by this command into the given
    * {@code selectionMatrixToPack}.
    * 
    * @param selectionMatrixToPack the selection matrix to pack.
    */
   public void getSelectionMatrix(SelectionMatrix6D selectionMatrixToPack)
   {
      selectionMatrixToPack.set(selectionMatrix);
   }

   /**
    * Gets the reference to the desired rate of change of momentum carried by this command.
    * <p>
    * It represents the desired rate of change of momentum at the center of mass while the data is
    * expressed in world frame.
    * </p>
    * 
    * @return the desired rate of change of momentum.
    */
   public DenseMatrix64F getMomentumRate()
   {
      return momentumRateOfChange;
   }

   /**
    * Packs the value of the desired rate of change of momentum into two frame vectors.
    * 
    * @param angularPartToPack frame vector to pack the desired rate of change of angular momentum
    *           at the center of mass. Modified.
    * @param linearPartToPack frame vector to pack the desired rate of change of linear momentum.
    *           Modified.
    */
   public void getMomentumRate(FrameVector angularPartToPack, FrameVector linearPartToPack)
   {
      angularPartToPack.setIncludingFrame(worldFrame, 0, momentumRateOfChange);
      linearPartToPack.setIncludingFrame(worldFrame, 3, momentumRateOfChange);
   }

   /**
    * {@inheritDoc}
    * 
    * @return {@link ControllerCoreCommandType#TASKSPACE}.
    */
   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.MOMENTUM;
   }

   @Override
   public String toString()
   {
      String stringOfMomentumRate = EuclidCoreIOTools.getStringOf("(", ")", ",", momentumRateOfChange.getData());
      return getClass().getSimpleName() + ": momentum rate: " + stringOfMomentumRate + ", selection matrix = " + selectionMatrix;
   }
}
