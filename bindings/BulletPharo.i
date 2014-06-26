// Bullet bindings interface definition file
%module Bullet

// Include Files
%{
#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"
%}

// Some macros
#define SIMD_FORCE_INLINE inline
#define ATTRIBUTE_ALIGNED16(a) a
#define ATTRIBUTE_ALIGNED64(a) a
#define ATTRIBUTE_ALIGNED128(a) a
#define BT_DECLARE_ALIGNED_ALLOCATOR()

typedef float btScalar;

%rename(BTTypedObject) btTypedObject;

struct btTypedObject
{
	btTypedObject(int objectType)
		:m_objectType(objectType)
	{
	}
	int	m_objectType;
	inline int getObjectType() const
	{
		return m_objectType;
	}
};

// Class renaming
%rename(BTQuadWord) btQuadWord;
%rename(BTVector3) btVector3;
%rename(BTQuaternion) btQuaternion;
%rename(BTMatrix3x3) btMatrix3x3;
%rename(BTTransform) btTransform;
%rename(BTTransformFloatData) btTransformFloatData;
%rename(BTTransformDoubleData) btTransformDoubleData;
%rename(BTMotionState) btMotionState;
%rename(BTDefaultMotionState) btDefaultMotionState;
%rename(BTClock) btClock;
%rename(BTIDebugDraw) btIDebugDraw;
%rename(BTChunk) btChunk;
%rename(BTSerializer) btSerializer;
%rename(BTPointerUid) btPointerUid;
%rename(BTDefaultSerializer) btDefaultSerializer;
%rename(BTCollisionWorld) btCollisionWorld;
%rename(BTCollisionObject) btCollisionObject;
%rename(BTCollisionObjectDoubleData) btCollisionObjectDoubleData;
%rename(BTCollisionObjectFloatData) btCollisionObjectFloatData;
%rename(BTConvexInternalShape) btConvexInternalShape;
%rename(BTConvexInternalShapeData) btConvexInternalShapeData;
%rename(BTConvexInternalAabbCachingShape) btConvexInternalAabbCachingShape;
%rename(BTPolyhedralConvexShape) btPolyhedralConvexShape;
%rename(BTPolyhedralConvexAabbCachingShape) btPolyhedralConvexAabbCachingShape;
%rename(BTBroadphaseProxy) btBroadphaseProxy;
%rename(BTBroadphasePair) btBroadphasePair;
%rename(BTBroadphasePairSortPredicate) btBroadphasePairSortPredicate;
%rename(BTBoxShape) btBoxShape;
%rename(BTSphereShape) btSphereShape;
%rename(BTCapsuleShape) btCapsuleShape;
%rename(BTCapsuleShapeX) btCapsuleShapeX;
%rename(BTCapsuleShapeZ) btCapsuleShapeZ;
%rename(BTCapsuleShapeData) btCapsuleShapeData;
%rename(BTCylinderShape) btCylinderShape;
%rename(BTCylinderShapeX) btCylinderShapeX;
%rename(BTCylinderShapeZ) btCylinderShapeZ;
%rename(BTCylinderShapeData) btCylinderShapeData;
%rename(BTConeShape) btConeShape;
%rename(BTConeShapeX) btConeShapeX;
%rename(BTConeShapeZ) btConeShapeZ;
%rename(BTConeShapeData) btConeShapeData;
%rename(BTConcaveShape) btConcaveShape;
%rename(BTStaticPlaneShape) btStaticPlaneShape;
%rename(BTStaticPlaneShapeData) btStaticPlaneShapeData;
%rename(BTConvexHullShape) btConvexHullShape;
%rename(BTConvexHullShapeData) btConvexHullShapeData;
%rename(BTTriangleMesh) btTriangleMesh;
%rename(BTConvexTriangleMeshShape) btConvexTriangleMeshShape;
%rename(BTBvhTriangleMeshShape) btBvhTriangleMeshShape;
%rename(BTTriangleMeshShapeData) btTriangleMeshShapeData;
%rename(BTScaledBvhTriangleMeshShape) btScaledBvhTriangleMeshShape;
%rename(BTScaledTriangleMeshShapeData) btScaledTriangleMeshShapeData;
%rename(BTTriangleMeshShape) btTriangleMeshShape;
%rename(BTStridingMeshInterface) btStridingMeshInterface;
%rename(BTIntIndexData) btIntIndexData;
%rename(BTShortIntIndexData) btShortIntIndexData;
%rename(BTShortIntIndexTripletData) btShortIntIndexTripletData;
%rename(BTCharIndexTripletData) btCharIndexTripletData;
%rename(BTMeshPartData) btMeshPartData;
%rename(BTStridingMeshInterfaceData) btStridingMeshInterfaceData;
%rename(BTIndexedMesh) btIndexedMesh;
%rename(BTTriangleIndexVertexArray) btTriangleIndexVertexArray;
%rename(BTCompoundShapeChild) btCompoundShapeChild;
%rename(BTCompoundShape) btCompoundShape;
%rename(BTCompoundShapeChildData) btCompoundShapeChildData;
%rename(BTCompoundShapeData) btCompoundShapeData;
%rename(BTBU_Simplex1to4) btBU_Simplex1to4;
%rename(BTEmptyShape) btEmptyShape;
%rename(BTMultiSphereShape) btMultiSphereShape;
%rename(BTPositionAndRadius) btPositionAndRadius;
%rename(BTMultiSphereShapeData) btMultiSphereShapeData;
%rename(BTUniformScalingShape) btUniformScalingShape;
%rename(BTCollisionAlgorithmCreateFunc) btCollisionAlgorithmCreateFunc;
%rename(BTSphereSphereCollisionAlgorithm) btSphereSphereCollisionAlgorithm;
%rename(BTDefaultCollisionConstructionInfo) btDefaultCollisionConstructionInfo;
%rename(BTDefaultCollisionConfiguration) btDefaultCollisionConfiguration;
%rename(BTDispatcherInfo) btDispatcherInfo;
%rename(BTDispatcher) btDispatcher;
%rename(BTActivatingCollisionAlgorithm) btActivatingCollisionAlgorithm;
%rename(BTCollisionDispatcher) btCollisionDispatcher;
%rename(BTSimpleBroadphaseProxy) btSimpleBroadphaseProxy;
%rename(BTSimpleBroadphase) btSimpleBroadphase;
%rename(BTAxisSweep3) btAxisSweep3;
%rename(BT32BitAxisSweep3) bt32BitAxisSweep3;
%rename(BTMultiSapBroadphase) btMultiSapBroadphase;
%rename(BTDbvtProxy) btDbvtProxy;
%rename(BTDbvtBroadphase) btDbvtBroadphase;
%rename(BTCollisionShape) btCollisionShape;
%rename(BTCollisionShapeData) btCollisionShapeData;
%rename(BTConvexShape) btConvexShape;
%rename(BTCollisionAlgorithmConstructionInfo) btCollisionAlgorithmConstructionInfo;
%rename(BTCollisionAlgorithm) btCollisionAlgorithm;
%rename(BTCollisionConfiguration) btCollisionConfiguration;
%rename(BTBroadphaseAabbCallback) btBroadphaseAabbCallback;
%rename(BTBroadphaseRayCallback) btBroadphaseRayCallback;
%rename(BTBroadphaseInterface) btBroadphaseInterface;

%rename(BTVector4) btVector4;
%rename(BTVector3FloatData) btVector3FloatData;
%rename(BTVector3DoubleData) btVector3DoubleData;
%rename(BTMatrix3x3FloatData) btMatrix3x3FloatData;
%rename(BTMatrix3x3DoubleData) btMatrix3x3DoubleData;
%rename(BTDynamicsWorld) btDynamicsWorld;
%rename(BTDynamicsWorldDoubleData) btDynamicsWorldDoubleData;
%rename(BTDynamicsWorldFloatData) btDynamicsWorldFloatData;
%rename(BTDiscreteDynamicsWorld) btDiscreteDynamicsWorld;
%rename(BTSimpleDynamicsWorld) btSimpleDynamicsWorld;
%rename(BTRigidBody) btRigidBody;
%rename(BTRigidBodyFloatData) btRigidBodyFloatData;
%rename(BTRigidBodyDoubleData) btRigidBodyDoubleData;
%rename(BTConstraintSetting) btConstraintSetting;
%rename(BTPoint2PointConstraint) btPoint2PointConstraint;
%rename(BTPoint2PointConstraintFloatData) btPoint2PointConstraintFloatData;
%rename(BTPoint2PointConstraintDoubleData2) btPoint2PointConstraintDoubleData2;
%rename(BTHingeConstraint) btHingeConstraint;
%rename(BTHingeConstraintFloatData) btHingeConstraintFloatData;
%rename(BTHingeConstraintDoubleData2) btHingeConstraintDoubleData2;
%rename(BTConeTwistConstraint) btConeTwistConstraint;
%rename(BTConeTwistConstraintDoubleData) btConeTwistConstraintDoubleData;
%rename(BTRotationalLimitMotor) btRotationalLimitMotor;
%rename(BTTranslationalLimitMotor) btTranslationalLimitMotor;
%rename(BTGeneric6DofConstraint) btGeneric6DofConstraint;
%rename(BTGeneric6DofConstraintData) btGeneric6DofConstraintData;
%rename(BTGeneric6DofConstraintDoubleData2) btGeneric6DofConstraintDoubleData2;
%rename(BTSliderConstraint) btSliderConstraint;
%rename(BTSliderConstraintData) btSliderConstraintData;
%rename(BTSliderConstraintDoubleData) btSliderConstraintDoubleData;
%rename(BTGeneric6DofSpringConstraint) btGeneric6DofSpringConstraint;
%rename(BTGeneric6DofSpringConstraintData) btGeneric6DofSpringConstraintData;
%rename(BTGeneric6DofSpringConstraintDoubleData2) btGeneric6DofSpringConstraintDoubleData2;
%rename(BTUniversalConstraint) btUniversalConstraint;
%rename(BTHinge2Constraint) btHinge2Constraint;
%rename(BTGearConstraint) btGearConstraint;
%rename(BTGearConstraintFloatData) btGearConstraintFloatData;
%rename(BTGearConstraintDoubleData) btGearConstraintDoubleData;
%rename(BTFixedConstraint) btFixedConstraint;
%rename(BTConstraintSolver) btConstraintSolver;
%rename(BTSequentialImpulseConstraintSolver) btSequentialImpulseConstraintSolver;
%rename(BTRaycastVehicle) btRaycastVehicle;
%rename(BTDefaultVehicleRaycaster) btDefaultVehicleRaycaster;

%rename(BTActionInterface) btActionInterface;
%rename(BTJointFeedback) btJointFeedback;
%rename(BTTypedConstraint) btTypedConstraint;
%rename(BTTypedConstraintFloatData) btTypedConstraintFloatData;
%rename(BTTypedConstraintData) btTypedConstraintData;
%rename(BTTypedConstraintDoubleData) btTypedConstraintDoubleData;
%rename(BTAngularLimit) btAngularLimit;
%rename(BTPoint2PointConstraintDoubleData) btPoint2PointConstraintDoubleData;
%rename(BTHingeConstraintDoubleData) btHingeConstraintDoubleData;
%rename(BTConeTwistConstraintData) btConeTwistConstraintData;
%rename(BTWheelInfoConstructionInfo) btWheelInfoConstructionInfo;
%rename(BTWheelInfo) btWheelInfo;
%rename(BTVehicleRaycaster) btVehicleRaycaster;

//%template(BTAxisSweep3InternalUnsignedShort) btAxisSweep3Internal<unsigned short int>;
//%template(BTAxisSweep3InternalUnsignedInt) btAxisSweep3Internal<unsigned int>;

///Math library & Utils
%include "LinearMath/btVector3.h"
%include "LinearMath/btQuadWord.h"
%include "LinearMath/btQuaternion.h"
%include "LinearMath/btMatrix3x3.h"
%include "LinearMath/btTransform.h"
%include "LinearMath/btMotionState.h"
%include "LinearMath/btDefaultMotionState.h"
%include "LinearMath/btIDebugDraw.h"
%include "LinearMath/btSerializer.h"

// Include the bullet files.
// Taken from btBulletCollisionCommon

///Bullet's btCollisionWorld and btCollisionObject definitions
%include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
%include "BulletCollision/CollisionDispatch/btCollisionObject.h"

///Collision Shapes
%include "BulletCollision/CollisionShapes/btCollisionShape.h"

%include "BulletCollision/CollisionShapes/btConvexShape.h"
%include "BulletCollision/CollisionShapes/btConvexInternalShape.h"
%include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
%include "BulletCollision/CollisionShapes/btCollisionMargin.h"
%include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"

%include "BulletCollision/CollisionShapes/btBoxShape.h"
%include "BulletCollision/CollisionShapes/btSphereShape.h"
%include "BulletCollision/CollisionShapes/btCapsuleShape.h"
%include "BulletCollision/CollisionShapes/btCylinderShape.h"
%include "BulletCollision/CollisionShapes/btConeShape.h"
%include "BulletCollision/CollisionShapes/btConcaveShape.h"

%include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
%include "BulletCollision/CollisionShapes/btConvexHullShape.h"
%include "BulletCollision/CollisionShapes/btStridingMeshInterface.h"
%include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
%include "BulletCollision/CollisionShapes/btTriangleMesh.h"
%include "BulletCollision/CollisionShapes/btTriangleMeshShape.h"
%include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
%include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
%include "BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h"
%include "BulletCollision/CollisionShapes/btCompoundShape.h"
%include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
%include "BulletCollision/CollisionShapes/btEmptyShape.h"
%include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
%include "BulletCollision/CollisionShapes/btUniformScalingShape.h"

///Narrowphase Collision Detector
%include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
%include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
%include "BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h"
%include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"

//%include "BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.h"
%include "BulletCollision/CollisionDispatch/btCollisionConfiguration.h"
%include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"

///Dispatching and generation of collision pairs (broadphase)
%include "BulletCollision/BroadphaseCollision/btDispatcher.h"

%include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
%include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
%include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
%include "BulletCollision/BroadphaseCollision/btAxisSweep3.h"
%include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"

// Dynamics
%include "BulletDynamics/Dynamics/btActionInterface.h"
%include "BulletDynamics/Dynamics/btDynamicsWorld.h"
%include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

%include "BulletDynamics/Dynamics/btSimpleDynamicsWorld.h"
%include "BulletDynamics/Dynamics/btRigidBody.h"

%include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
%include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"
%include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
%include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
%include "BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
%include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
%include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
%include "BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h"
%include "BulletDynamics/ConstraintSolver/btUniversalConstraint.h"
%include "BulletDynamics/ConstraintSolver/btHinge2Constraint.h"
%include "BulletDynamics/ConstraintSolver/btGearConstraint.h"
%include "BulletDynamics/ConstraintSolver/btFixedConstraint.h"


%include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"


///Vehicle simulation, with wheel contact simulated by raycasts
//%include "BulletDynamics/Vehicle/btWheelInfo.h"
//%include "BulletDynamics/Vehicle/btVehicleRaycaster.h"
//%include "BulletDynamics/Vehicle/btRaycastVehicle.h"

