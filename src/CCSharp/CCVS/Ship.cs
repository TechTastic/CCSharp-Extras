using CCSharp.Attributes;
using CCSharp.ComputerCraft;
using CCSharp.AdvancedMath;

namespace CCSharp.CCVS;

public class Ship
{
    [LuaMethod("ship.getId")]
    public static Long GetId() => default;

    [LuaMethod("ship.getSlug")]
    public static string GetSlug() => default;

    [LuaMethod("ship.setSlug")]
    public static void SetSlug(string slug) => default;

    [LuaMethod("ship.getMass")]
    public static double GetMass() => default;

    [LuaMethod("ship.isStatic")]
    public static bool IsStatic() => default;

    [LuaMethod("ship.getConstraints")]
    public static Dictionary<Long, Constraint> GetConstraints() => default;

    [LuaMethod("ship.getShipyardPosition")]
    public static Vector3 GetShipyardPosition() => default;

    [LuaMethod("ship.getWorldspacePosition")]
    public static Vector3 GetWorldspacePosition() => default;

    [LuaMethod("ship.getVelocity")]
    public static Vector3 GetLinearVelocity() => default;

    [LuaMethod("ship.getOmega")]
    public static Vector3 GetAngularVelocity() => default;

    [LuaMethod("ship.getScale")]
    public static Vector3 GetScale() => default;

    [LuaMethod("ship.setScale")]
    public static void SetScale(double scale) => default;

    [LuaMethod("ship.getQuaternion")]
    public static Quaternion GetQuaternion() => default;

    [LuaMethod("ship.getTransformationMatrix")]
    public static Matrix GetTransformationMatrix() => default;

    [LuaMethod("ship.getMomentOfInertiaTensorToSave")]
    public static Matrix GetMomentOfInertiaTensorToSave() => default;

    [LuaMethod("ship.getMomentOfInertiaTensor")]
    public static Matrix GetMomentOfInertiaTensor() => default;

    [LuaMethod("ship.transformPositionToWorld")]
    public static Vector3 transformPositionToWorld(Vector3 pos) => default;

    [LuaMethod("ship.pullPhysicsTicks")]
    public static (string name, PhysicsTick[] ticks) PullPhysicsTicks() => default;

    [LuaTableTypeCheck(TableAccessor = "getBuoyantFactor"), LuaTableTypeCheck(TableAccessor = "isStatic"), LuaTableTypeCheck(TableAccessor = "doFluidDrag"), LuaTableTypeCheck(TableAccessor = "getInertia"), LuaTableTypeCheck(TableAccessor = "getPoseVel"), LuaTableTypeCheck(TableAccessor = "getForceInducers")]
    class PhysicsTick
    {
        [LuaMethod("getBuoyantFactor")]
        public double GetBuoyantFactor() => default;

        [LuaMethod("isStatic")]
        public bool IsStatic() => default;

        [LuaMethod("doFluidDrag")]
        public bool FluidDrag() => default;

        [LuaMethod("getInertia")]
        public Inertia GetInertia() => default;

        [LuaMethod("getPoseVel")]
        public PoseVel GetPoseVel() => default;

        [LuaMethod("getForceInducers")]
        public string[] GetForceInducers() => default;

        class Inertia
        {
            [LuaProperty("momentOfInertiaTensor")] public Matrix MomentOfInertiaTensor { get; set; }
            [LuaProperty("mass")] public double Mass { get; set; }
        }

        class PoseVel
        {
            [LuaProperty("vel")] public Vector3 LinearVelocity { get; set; }
            [LuaProperty("omega")] public Vector3 AngularVelocity { get; set; }
            [LuaProperty("pos")] public Vector3 CenterOfMass { get; set; }
            [LuaProperty("rot")] public Quaternion Rotation { get; set; }
        }

    }

    [LuaMethod("ship.teleport")]
    public static void Teleport(object data) => default;

    [LuaMethod("ship.applyInvariantForce")]
    public static void ApplyInvariantForce(Vector3 force) => default;
    [LuaMethod("ship.applyInvariantForce")]
    public static void ApplyInvariantForce(double forceX, double forceY, double forceZ) => default;

    [LuaMethod("ship.applyInvariantTorque")]
    public static void ApplyInvariantTorque(Vector3 torque) => default;
    [LuaMethod("ship.applyInvariantTorque")]
    public static void ApplyInvariantTorque(double torqueX, double torqueY, double torqueZ) => default;

    [LuaMethod("ship.applyInvariantForceToPos")]
    public static void ApplyInvariantForceToPosition(Vector3 force, Vector3 pos) => default;
    [LuaMethod("ship.applyInvariantForceToPos")]
    public static void ApplyInvariantForceToPosition(double forceX, double forceY, double forceZ, double posX, double posY, double posZ) => default;

    [LuaMethod("ship.applyRotDependentForce")]
    public static void ApplyRotDependentForce(Vector3 force) => default;
    [LuaMethod("ship.applyRotDependentForce")]
    public static void ApplyRotDependentForce(double forceX, double forceY, double forceZ) => default;

    [LuaMethod("ship.applyRotDependentTorque")]
    public static void ApplyRotDependentTorque(Vector3 torque) => default;
    [LuaMethod("ship.applyRotDependentTorque")]
    public static void ApplyRotDependentTorque(double torqueX, double torqueY, double torqueZ) => default;

    [LuaMethod("ship.applyRotDependentForceToPos")]
    public static void ApplyRotDependentForceToPosition(Vector3 force, Vector3 pos) => default;
    [LuaMethod("ship.applyRotDependentForceToPos")]
    public static void ApplyRotDependentForceToPosition(double forceX, double forceY, double forceZ, double posX, double posY, double posZ) => default;

    [LuaTableTypeCheck(TableAccessor = "type")]
    class Constraint
    {
        [LuaEnum(typeof(ConstraintType))]
        public enum ConstraintType
        {
            [LuaEnumValue("attachment")] Attachment,
            [LuaEnumValue("fixed_attachment_orientation")] FixedAttachmentOrientation,
            [LuaEnumValue("fixed_orientation")] FixedOrientation,
            [LuaEnumValue("hinge_orientation")] HingeOrientation,
            [LuaEnumValue("hinge_swing_limits")] HingeSwingLimits,
            [LuaEnumValue("hinge_target_angle")] HingeTargetAngle,
            [LuaEnumValue("pos_damping")] PosDamping,
            [LuaEnumValue("rope")] Rope,
            [LuaEnumValue("rot_damping")] RotDamping,
            [LuaEnumValue("Slide")] Slide,
            [LuaEnumValue("spherical_swing_limits")] SphericalSwingLimits,
            [LuaEnumValue("spherical_twist_limits")] SphericalTwistLimits
        }

        [LuaProperty("shipId0")] public Long FirstShipID { get; set; }
        [LuaProperty("shipId1")] public Long SecondShipID { get; set; }
        [LuaProperty("type")] public ConstraintType Type { get; set; }
        [LuaProperty("compliance")] public double Compliance { get; set; }
    }

    [LuaImplicitTypeArgument("type")]
    class AttachmentConstraint : Constraint
    {
        [LuaProperty("localPos0")] public Vector3 FirstPosition { get; set; }
        [LuaProperty("localPos1")] public Vector3 SecondPosition { get; set; }
        [LuaProperty("maxForce")] public double MaxForce { get; set; }
        [LuaProperty("fixedDistance")] public double FixedDistance { get; set; }
    }

    [LuaImplicitTypeArgument("type")]
    class HingeSwingLimitsConstraint : Constraint
    {
        [LuaProperty("localRot0")] public Quaternion FirstPosition { get; set; }
        [LuaProperty("localRot1")] public Quaternion SecondPosition { get; set; }
        [LuaProperty("maxTorque")] public double MaxTorque { get; set; }
        [LuaProperty("minSwingAngle")] public double MinSwingAngle { get; set; }
        [LuaProperty("maxSwingAngle")] public double MaxSwingAngle { get; set; }
    }

    [LuaImplicitTypeArgument("type")]
    class HingeTargetAngleConstraint : Constraint
    {
        [LuaProperty("localRot0")] public Quaternion FirstPosition { get; set; }
        [LuaProperty("localRot1")] public Quaternion SecondPosition { get; set; }
        [LuaProperty("maxTorque")] public double MaxTorque { get; set; }
        [LuaProperty("targetAngle")] public double TargetAngle { get; set; }
        [LuaProperty("nextTickTargetAngle")] public double NextTickTargetAngle { get; set; }
    }

    [LuaImplicitTypeArgument("type")]
    class PosDampingConstraint : Constraint
    {
        [LuaProperty("localPos0")] public Vector3 FirstPosition { get; set; }
        [LuaProperty("localPos1")] public Vector3 SecondPosition { get; set; }
        [LuaProperty("maxForce")] public double MaxForce { get; set; }
        [LuaProperty("posDamping")] public double PosDamping { get; set; }
    }

    [LuaImplicitTypeArgument("type")]
    class RopeConstraint : Constraint
    {
        [LuaProperty("localPos0")] public Vector3 FirstPosition { get; set; }
        [LuaProperty("localPos1")] public Vector3 SecondPosition { get; set; }
        [LuaProperty("maxForce")] public double MaxForce { get; set; }
        [LuaProperty("ropeLength")] public double RopeLength { get; set; }
    }

    [LuaImplicitTypeArgument("type")]
    class RotDampingConstraint : Constraint
    {
        [LuaEnum(typeof(RotDampingAxes))]
        public enum RotDampingAxes
        {
            [LuaEnumValue("parallel")] Parallel,
            [LuaEnumValue("perpendicular")] Perpendicular,
            [LuaEnumValue("all_axes")] AllAxes
        }

        [LuaProperty("localPos0")] public Vector3 FirstPosition { get; set; }
        [LuaProperty("localPos1")] public Vector3 SecondPosition { get; set; }
        [LuaProperty("maxForce")] public double MaxForce { get; set; }
        [LuaProperty("rotDamping")] public double RotDamping { get; set; }
        [LuaProperty("rotDampingAxes")] public RotDampingAxes RotDampingAxis { get; set; }
    }

    [LuaImplicitTypeArgument("type")]
    class RotDampingConstraint : Constraint
    {
        [LuaProperty("localPos0")] public Vector3 FirstPosition { get; set; }
        [LuaProperty("localPos1")] public Vector3 SecondPosition { get; set; }
        [LuaProperty("maxForce")] public double MaxForce { get; set; }
        [LuaProperty("localSlideAxis0")] public Vector3 LocalSlideAxis { get; set; }
        [LuaProperty("maxDistBetweenPoints")] public double MaxDistanceBetweenPoints { get; set; }
    }

    [LuaImplicitTypeArgument("type")]
    class SphericalSwingLimitsConstraint : Constraint
    {
        [LuaProperty("localRot0")] public Quaternion FirstPosition { get; set; }
        [LuaProperty("localRot1")] public Quaternion SecondPosition { get; set; }
        [LuaProperty("maxTorque")] public double MaxTorque { get; set; }
        [LuaProperty("minSwingAngle")] public double MinSwingAngle { get; set; }
        [LuaProperty("maxSwingAngle")] public double MaxSwingAngle { get; set; }
    }

    [LuaImplicitTypeArgument("type")]
    class SphericalTwistLimitsConstraint : Constraint
    {
        [LuaProperty("localRot0")] public Quaternion FirstPosition { get; set; }
        [LuaProperty("localRot1")] public Quaternion SecondPosition { get; set; }
        [LuaProperty("maxTorque")] public double MaxTorque { get; set; }
        [LuaProperty("minTwistAngle")] public double MinTwistAngle { get; set; }
        [LuaProperty("maxTwistAngle")] public double MaxTwistAngle { get; set; }
    }
}