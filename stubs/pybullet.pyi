from typing import Any
from typing import overload

# Activation States
ACTIVATION_STATE_DISABLE_SLEEPING: int
ACTIVATION_STATE_DISABLE_WAKEUP: int
ACTIVATION_STATE_ENABLE_SLEEPING: int
ACTIVATION_STATE_ENABLE_WAKEUP: int
ACTIVATION_STATE_SLEEP: int
ACTIVATION_STATE_WAKE_UP: int

# File IO Actions
AddFileIOAction: int

# Keyboard Keys
B3G_ALT: int
B3G_BACKSPACE: int
B3G_CONTROL: int
B3G_DELETE: int
B3G_DOWN_ARROW: int
B3G_END: int
B3G_F1: int
B3G_F10: int
B3G_F11: int
B3G_F12: int
B3G_F13: int
B3G_F14: int
B3G_F15: int
B3G_F2: int
B3G_F3: int
B3G_F4: int
B3G_F5: int
B3G_F6: int
B3G_F7: int
B3G_F8: int
B3G_F9: int
B3G_HOME: int
B3G_INSERT: int
B3G_LEFT_ARROW: int
B3G_PAGE_DOWN: int
B3G_PAGE_UP: int
B3G_RETURN: int
B3G_RIGHT_ARROW: int
B3G_SHIFT: int
B3G_SPACE: int
B3G_UP_ARROW: int

# File IO
CNSFileIO: int

# Constraint Solvers
CONSTRAINT_SOLVER_LCP_DANTZIG: int
CONSTRAINT_SOLVER_LCP_PGS: int
CONSTRAINT_SOLVER_LCP_SI: int

# Contact Flags
CONTACT_RECOMPUTE_CLOSEST: int
CONTACT_REPORT_EXISTING: int

# Visualization Options
COV_ENABLE_DEPTH_BUFFER_PREVIEW: int
COV_ENABLE_GUI: int
COV_ENABLE_KEYBOARD_SHORTCUTS: int
COV_ENABLE_MOUSE_PICKING: int
COV_ENABLE_PLANAR_REFLECTION: int
COV_ENABLE_RENDERING: int
COV_ENABLE_RGB_BUFFER_PREVIEW: int
COV_ENABLE_SEGMENTATION_MARK_PREVIEW: int
COV_ENABLE_SHADOWS: int
COV_ENABLE_SINGLE_STEP_RENDERING: int
COV_ENABLE_TINY_RENDERER: int
COV_ENABLE_VR_PICKING: int
COV_ENABLE_VR_RENDER_CONTROLLERS: int
COV_ENABLE_VR_TELEPORTING: int
COV_ENABLE_WIREFRAME: int
COV_ENABLE_Y_AXIS_UP: int

# Connection Methods
DIRECT: int
ER_BULLET_HARDWARE_OPENGL: int
ER_NO_SEGMENTATION_MASK: int
ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX: int
ER_TINY_RENDERER: int
ER_USE_PROJECTIVE_TEXTURE: int

# Geometric Types
GEOM_BOX: int
GEOM_CAPSULE: int
GEOM_CONCAVE_INTERNAL_EDGE: int
GEOM_CYLINDER: int
GEOM_FORCE_CONCAVE_TRIMESH: int
GEOM_HEIGHTFIELD: int
GEOM_MESH: int
GEOM_PLANE: int
GEOM_SPHERE: int

# Graphics
GRAPHICS_CLIENT: int
GRAPHICS_SERVER: int
GRAPHICS_SERVER_MAIN_THREAD: int
GRAPHICS_SERVER_TCP: int

# GUI
GUI: int
GUI_MAIN_THREAD: int
GUI_SERVER: int

# Inverse Kinematics
IK_DLS: int
IK_HAS_JOINT_DAMPING: int
IK_HAS_NULL_SPACE_VELOCITY: int
IK_HAS_TARGET_ORIENTATION: int
IK_HAS_TARGET_POSITION: int
IK_SDLS: int

# Joint Feedback
JOINT_FEEDBACK_IN_JOINT_FRAME: int
JOINT_FEEDBACK_IN_WORLD_SPACE: int

# Joint Types
JOINT_FIXED: int
JOINT_GEAR: int
JOINT_PLANAR: int
JOINT_POINT2POINT: int
JOINT_PRISMATIC: int
JOINT_REVOLUTE: int
JOINT_SPHERICAL: int

# Key States
KEY_IS_DOWN: int
KEY_WAS_RELEASED: int
KEY_WAS_TRIGGERED: int

# Frames
LINK_FRAME: int
WORLD_FRAME: int

# Mesh Data
MESH_DATA_SIMULATION_MESH: int
MJCF_COLORS_FROM_FILE: int

# Control Modes
PD_CONTROL: int
POSITION_CONTROL: int
TORQUE_CONTROL: int
VELOCITY_CONTROL: int

# File IO Types
PosixFileIO: int
ZipFileIO: int

# Reset Flags
RESET_USE_DEFORMABLE_WORLD: int
RESET_USE_DISCRETE_DYNAMICS_WORLD: int
RESET_USE_REDUCED_DEFORMABLE_WORLD: int
RESET_USE_SIMPLE_BROADPHASE: int

# State Logging
STATE_LOGGING_ALL_COMMANDS: int
STATE_LOGGING_CONTACT_POINTS: int
STATE_LOGGING_CUSTOM_TIMER: int
STATE_LOGGING_GENERIC_ROBOT: int
STATE_LOGGING_MINITAUR: int
STATE_LOGGING_PROFILE_TIMINGS: int
STATE_LOGGING_VIDEO_MP4: int
STATE_LOGGING_VR_CONTROLLERS: int
STATE_LOG_JOINT_MOTOR_TORQUES: int
STATE_LOG_JOINT_TORQUES: int
STATE_LOG_JOINT_USER_TORQUES: int
STATE_REPLAY_ALL_COMMANDS: int

# Communication Protocols
TCP: int
UDP: int

# URDF Flags
URDF_ENABLE_CACHED_GRAPHICS_SHAPES: int
URDF_ENABLE_SLEEPING: int
URDF_ENABLE_WAKEUP: int
URDF_GLOBAL_VELOCITIES_MB: int
URDF_GOOGLEY_UNDEFINED_COLORS: int
URDF_IGNORE_COLLISION_SHAPES: int
URDF_IGNORE_VISUAL_SHAPES: int
URDF_INITIALIZE_SAT_FEATURES: int
URDF_MAINTAIN_LINK_ORDER: int
URDF_MERGE_FIXED_LINKS: int
URDF_PRINT_URDF_INFO: int
URDF_USE_IMPLICIT_CYLINDER: int
URDF_USE_INERTIA_FROM_FILE: int
URDF_USE_MATERIAL_COLORS_FROM_MTL: int
URDF_USE_MATERIAL_TRANSPARANCY_FROM_MTL: int
URDF_USE_SELF_COLLISION: int
URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS: int
URDF_USE_SELF_COLLISION_EXCLUDE_PARENT: int
URDF_USE_SELF_COLLISION_INCLUDE_PARENT: int

# Visual Shape Data
VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS: int
VISUAL_SHAPE_DOUBLE_SIDED: int

# VR States
VR_BUTTON_IS_DOWN: int
VR_BUTTON_WAS_RELEASED: int
VR_BUTTON_WAS_TRIGGERED: int
VR_CAMERA_TRACK_OBJECT_ORIENTATION: int
VR_DEVICE_CONTROLLER: int
VR_DEVICE_GENERIC_TRACKER: int
VR_DEVICE_HMD: int
VR_MAX_BUTTONS: int
VR_MAX_CONTROLLERS: int

class error(Exception):
    """Generic error class."""

def addUserData(bodyUniqueId, key, value, linkIndex = ..., visualShapeIndex = ..., physicsClientId = ...) -> Any:
    """Add user data to a body."""
def addUserDebugLine(*args, **kwargs) -> Any:
    """Add a debug line to the visualization."""
def addUserDebugParameter(*args, **kwargs) -> Any:
    """Add a user debug parameter slider."""
def addUserDebugPoints(*args, **kwargs) -> Any:
    """Add debug points to the visualization."""
def addUserDebugText(*args, **kwargs) -> Any:
    """Add debug text to the visualization."""
def applyExternalForce(*args, **kwargs) -> Any:
    """Apply an external force to a body."""
def applyExternalTorque(*args, **kwargs) -> Any:
    """Apply an external torque to a body."""
def calculateInverseDynamics(*args, **kwargs) -> Any:
    """Calculate inverse dynamics for a given body."""
def calculateInverseKinematics(*args, **kwargs) -> Any:
    """Calculate inverse kinematics for a given body."""
def calculateInverseKinematics2(*args, **kwargs) -> Any:
    """Calculate inverse kinematics using a secondary method."""
def calculateJacobian(bodyUniqueId, linkIndex, localPosition, objPositions, objVelocities, objAccelerations, physicsClientId = ...) -> Any:
    """Calculate the Jacobian for a given link."""
def calculateMassMatrix(bodyUniqueId, objPositions, physicsClientId = ...) -> Any:
    """Calculate the mass matrix for a given body."""
def calculateVelocityQuaternion(*args, **kwargs) -> Any:
    """Calculate the velocity quaternion."""
def changeConstraint(*args, **kwargs) -> Any:
    """Change the properties of an existing constraint."""
def changeDynamics(*args, **kwargs) -> Any:
    """Change the dynamics properties of a body."""
def changeTexture(*args, **kwargs) -> Any:
    """Change the texture of a visual shape."""
def changeVisualShape(*args, **kwargs) -> Any:
    """Change the visual properties of a shape."""
def computeDofCount(*args, **kwargs) -> Any:
    """Compute the degree of freedom count for a given body."""
def computeProjectionMatrix(*args, **kwargs) -> Any:
    """Compute a projection matrix."""
def computeProjectionMatrixFOV(*args, **kwargs) -> Any:
    """Compute a projection matrix with a field of view."""
def computeViewMatrix(*args, **kwargs) -> Any:
    """Compute a view matrix."""
def computeViewMatrixFromYawPitchRoll(*args, **kwargs) -> Any:
    """Compute a view matrix from yaw, pitch, and roll angles."""
def configureDebugVisualizer(*args, **kwargs) -> Any:
    """Configure the debug visualizer."""
@overload
def connect(method, key = ..., options = ...) -> Any:
    """Connect to a physics server using method, key, and options."""
@overload
def connect(method, hostname = ..., port = ..., options = ...) -> Any:
    """Connect to a physics server using method, hostname, port, and options."""
def createCollisionShape(*args, **kwargs) -> Any:
    """Create a collision shape."""
def createCollisionShapeArray(*args, **kwargs) -> Any:
    """Create an array of collision shapes."""
def createConstraint(*args, **kwargs) -> Any:
    """Create a constraint."""
def createMultiBody(*args, **kwargs) -> Any:
    """Create a multi-body system."""
def createSoftBodyAnchor(*args, **kwargs) -> Any:
    """Create an anchor for a soft body."""
def createVisualShape(*args, **kwargs) -> Any:
    """Create a visual shape."""
def createVisualShapeArray(*args, **kwargs) -> Any:
    """Create an array of visual shapes."""
def disconnect(physicsClientId = ...) -> Any:
    """Disconnect from the physics server."""
def enableJointForceTorqueSensor(*args, **kwargs) -> Any:
    """Enable force/torque sensor on a joint."""
def executePluginCommand(*args, **kwargs) -> Any:
    """Execute a command on a plugin."""
def getAABB(*args, **kwargs) -> Any:
    """Get the Axis-Aligned Bounding Box for a body or shape."""
def getAPIVersion(*args, **kwargs) -> Any:
    """Get the version of the API."""
def getAxisAngleFromQuaternion(*args, **kwargs) -> Any:
    """Get axis-angle representation from a quaternion."""
def getAxisDifferenceQuaternion(*args, **kwargs) -> Any:
    """Get the difference quaternion between two axes."""
def getBasePositionAndOrientation(*args, **kwargs) -> Any:
    """Get the base position and orientation of a body."""
def getBaseVelocity(*args, **kwargs) -> Any:
    """Get the base velocity of a body."""
def getBodyInfo(*args, **kwargs) -> Any:
    """Get information about a body."""
def getBodyUniqueId(*args, **kwargs) -> Any:
    """Get the unique ID of a body."""
def getCameraImage(*args, **kwargs) -> Any:
    """Capture an image from the camera."""
def getClosestPoints(*args, **kwargs) -> Any:
    """Get the closest points between two bodies."""
def getCollisionShapeData(*args, **kwargs) -> Any:
    """Get data about a collision shape."""
def getConnectionInfo(physicsClientId = ...) -> Any:
    """Get information about the current connection."""
def getConstraintInfo(*args, **kwargs) -> Any:
    """Get information about a constraint."""
def getConstraintState(*args, **kwargs) -> Any:
    """Get the state of a constraint."""
def getConstraintUniqueId(*args, **kwargs) -> Any:
    """Get the unique ID of a constraint."""
def getContactPoints(*args, **kwargs) -> Any:
    """Get contact points between bodies."""
def getDebugVisualizerCamera(*args, **kwargs) -> Any:
    """Get the debug visualizer camera settings."""
def getDifferenceQuaternion(*args, **kwargs) -> Any:
    """Get the difference quaternion between two quaternions."""
def getDynamicsInfo(*args, **kwargs) -> Any:
    """Get dynamics information for a body."""
def getEulerFromQuaternion(*args, **kwargs) -> Any:
    """Convert a quaternion to Euler angles."""
def getJointInfo(*args, **kwargs) -> Any:
    """Get information about a joint."""
def getJointState(*args, **kwargs) -> Any:
    """Get the state of a joint."""
def getJointStateMultiDof(*args, **kwargs) -> Any:
    """Get the state of a multi-DOF joint."""
def getJointStates(*args, **kwargs) -> Any:
    """Get the states of multiple joints."""
def getJointStatesMultiDof(*args, **kwargs) -> Any:
    """Get the states of multiple multi-DOF joints."""
def getKeyboardEvents(*args, **kwargs) -> Any:
    """Get keyboard events."""
def getLinkState(objectUniqueId, linkIndex, computeLinkVelocity = ..., computeForwardKinematics = ..., physicsClientId = ...) -> Any:
    """Get the state of a link."""
def getLinkStates(*args, **kwargs) -> Any:
    """Get the states of multiple links."""
def getMatrixFromQuaternion(*args, **kwargs) -> Any:
    """Get a rotation matrix from a quaternion."""
def getMeshData(*args, **kwargs) -> Any:
    """Get mesh data."""
def getMouseEvents(*args, **kwargs) -> Any:
    """Get mouse events."""
def getNumBodies(*args, **kwargs) -> Any:
    """Get the number of bodies in the simulation."""
def getNumConstraints(*args, **kwargs) -> Any:
    """Get the number of constraints in the simulation."""
def getNumJoints(*args, **kwargs) -> Any:
    """Get the number of joints in a body."""
def getNumUserData(bodyUniqueIdphysicsClientId = ...) -> Any:
    """Get the number of user data entries for a body."""
def getOverlappingObjects(*args, **kwargs) -> Any:
    """Get overlapping objects within a volume."""
def getPhysicsEngineParameters(*args, **kwargs) -> Any:
    """Get parameters of the physics engine."""
def getQuaternionFromAxisAngle(*args, **kwargs) -> Any:
    """Get a quaternion from an axis-angle representation."""
def getQuaternionFromEuler(*args, **kwargs) -> Any:
    """Get a quaternion from Euler angles."""
def getQuaternionSlerp(*args, **kwargs) -> Any:
    """Spherical linear interpolation between two quaternions."""
def getTetraMeshData(*args, **kwargs) -> Any:
    """Get tetrahedral mesh data."""
def getUserData(userDataId, physicsClientId = ...) -> Any:
    """Get user data by ID."""
def getUserDataId(bodyUniqueId, key, linkIndex = ..., visualShapeIndex = ..., physicsClientId = ...) -> Any:
    """Get user data ID by body ID and key."""
def getUserDataInfo(bodyUniqueId, userDataIndex, physicsClientId = ...) -> Any:
    """Get user data information."""
def getVREvents(*args, **kwargs) -> Any:
    """Get VR events."""
def getVisualShapeData(*args, **kwargs) -> Any:
    """Get visual shape data."""
def invertTransform(*args, **kwargs) -> Any:
    """Invert a transformation."""
def isConnected(physicsClientId = ...) -> Any:
    """Check if connected to a physics server."""
def isNumpyEnabled(*args, **kwargs) -> Any:
    """Check if numpy is enabled."""
def loadBullet(*args, **kwargs) -> Any:
    """Load a Bullet file."""
def loadMJCF(*args, **kwargs) -> Any:
    """Load an MJCF file."""
def loadPlugin(*args, **kwargs) -> Any:
    """Load a plugin."""
def loadSDF(*args, **kwargs) -> Any:
    """Load an SDF file."""
def loadSoftBody(*args, **kwargs) -> Any:
    """Load a soft body."""
def loadTexture(*args, **kwargs) -> Any:
    """Load a texture."""
def loadURDF(fileName, basePosition = ..., baseOrientation = ..., useMaximalCoordinates = ..., useFixedBase = ..., flags = ..., globalScaling = ..., physicsClientId = ...) -> Any:
    """Load a URDF file."""
def multiplyTransforms(*args, **kwargs) -> Any:
    """Multiply two transformations."""
def performCollisionDetection(physicsClientId = ...) -> Any:
    """Perform collision detection."""
def rayTest(*args, **kwargs) -> Any:
    """Perform a ray test."""
def rayTestBatch(*args, **kwargs) -> Any:
    """Perform a batch of ray tests."""
def readUserDebugParameter(*args, **kwargs) -> Any:
    """Read a user debug parameter."""
def removeAllUserDebugItems(*args, **kwargs) -> Any:
    """Remove all user debug items."""
def removeAllUserParameters(*args, **kwargs) -> Any:
    """Remove all user parameters."""
def removeBody(*args, **kwargs) -> Any:
    """Remove a body from the simulation."""
def removeCollisionShape(*args, **kwargs) -> Any:
    """Remove a collision shape."""
def removeConstraint(*args, **kwargs) -> Any:
    """Remove a constraint."""
def removeState(*args, **kwargs) -> Any:
    """Remove a state."""
def removeUserData(userDataId, physicsClientId = ...) -> Any:
    """Remove user data by ID."""
def removeUserDebugItem(*args, **kwargs) -> Any:
    """Remove a user debug item."""
def renderImage(*args, **kwargs) -> Any:
    """Render an image."""
def resetBasePositionAndOrientation(*args, **kwargs) -> Any:
    """Reset the base position and orientation of a body."""
def resetBaseVelocity(*args, **kwargs) -> Any:
    """Reset the base velocity of a body."""
def resetDebugVisualizerCamera(*args, **kwargs) -> Any:
    """Reset the debug visualizer camera."""
def resetJointState(objectUniqueId, jointIndex, targetValue, targetVelocity = ..., physicsClientId = ...) -> Any:
    """Reset the state of a joint."""
def resetJointStateMultiDof(objectUniqueId, jointIndex, targetValue, targetVelocity = ..., physicsClientId = ...) -> Any:
    """Reset the state of a multi-DOF joint."""
def resetJointStatesMultiDof(objectUniqueId, jointIndices, targetValues, targetVelocities = ..., physicsClientId = ...) -> Any:
    """Reset the states of multiple multi-DOF joints."""
def resetMeshData(*args, **kwargs) -> Any:
    """Reset mesh data."""
def resetSimulation(physicsClientId = ...) -> Any:
    """Reset the simulation."""
def resetVisualShapeData(*args, **kwargs) -> Any:
    """Reset visual shape data."""
def restoreState(*args, **kwargs) -> Any:
    """Restore a previously saved state."""
def rotateVector(*args, **kwargs) -> Any:
    """Rotate a vector."""
def saveBullet(*args, **kwargs) -> Any:
    """Save a Bullet file."""
def saveState(*args, **kwargs) -> Any:
    """Save the current state."""
def saveWorld(filename) -> Any:
    """Save the current world to a file."""
def setAdditionalSearchPath(*args, **kwargs) -> Any:
    """Set an additional search path."""
def setCollisionFilterGroupMask(*args, **kwargs) -> Any:
    """Set the collision filter group and mask."""
def setCollisionFilterPair(*args, **kwargs) -> Any:
    """Set the collision filter for a pair of bodies."""
def setDebugObjectColor(*args, **kwargs) -> Any:
    """Set the color of a debug object."""
def setDefaultContactERP(defaultContactERP, physicsClientId = ...) -> Any:
    """Set the default contact ERP."""
def setGravity(gravX, gravY, gravZ, physicsClientId = ...) -> Any:
    """Set the gravity vector."""
def setInternalSimFlags(*args, **kwargs) -> Any:
    """Set internal simulation flags."""
def setJointMotorControl(*args, **kwargs) -> Any:
    """Set the control mode of a joint."""
def setJointMotorControl2(*args, **kwargs) -> Any:
    """Set the control mode of a joint with advanced parameters."""
def setJointMotorControlArray(*args, **kwargs) -> Any:
    """Set the control mode of multiple joints."""
def setJointMotorControlMultiDof(*args, **kwargs) -> Any:
    """Set the control mode of a multi-DOF joint."""
def setJointMotorControlMultiDofArray(*args, **kwargs) -> Any:
    """Set the control mode of multiple multi-DOF joints."""
def setPhysicsEngineParameter(*args, **kwargs) -> Any:
    """Set parameters for the physics engine."""
def setRealTimeSimulation(enableRealTimeSimulation, physicsClientId = ...) -> Any:
    """Enable or disable real-time simulation."""
def setTimeOut(*args, **kwargs) -> Any:
    """Set the timeout for the simulation."""
def setTimeStep(timestep, physicsClientId = ...) -> Any:
    """Set the simulation timestep."""
def setVRCameraState(*args, **kwargs) -> Any:
    """Set the state of the VR camera."""
def startStateLogging(*args, **kwargs) -> Any:
    """Start state logging."""
def stepSimulation(physicsClientId = ...) -> Any:
    """Step the simulation."""
def stopStateLogging(*args, **kwargs) -> Any:
    """Stop state logging."""
def submitProfileTiming(*args, **kwargs) -> Any:
    """Submit profile timing."""
def syncBodyInfo(physicsClientId = ...) -> Any:
    """Synchronize body information."""
def syncUserData(bodyUniqueIds = ..., physicsClientId = ...) -> Any:
    """Synchronize user data."""
def unloadPlugin(*args, **kwargs) -> Any:
    """Unload a plugin."""
def unsupportedChangeScaling(*args, **kwargs) -> Any:
    """Unsupported change scaling."""
def vhacd(*args, **kwargs) -> Any:
    """Perform V-HACD (Volumetric Hierarchical Approximate Convex Decomposition)."""
