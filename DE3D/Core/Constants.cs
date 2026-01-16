/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Physics Constants for Unity3D        /____/                            
                                                                    By: Elitmers */
namespace DynamicEngine
{
    public static class PhysicsConstants
    {
        #region Collision Constants
        public const float MIN_COLLISION_DISTANCE = 0.001f;
        public const float DEFAULT_COLLISION_EPSILON = 0.001f;
        public const int MAX_OVERLAP_RESULTS = 32;
        public const float MESH_RAYCAST_DISTANCE_MULTIPLIER = 2f;
        public const float DEFAULT_TRIANGLE_THICKNESS = 0.02f;

        #endregion

        #region Constraint Constants
        public const float MIN_BEAM_LENGTH = 0.01f;
        public const float MAX_ALLOWED_DISPLACEMENT = 1.0f;
        public const float CONSTRAINT_EPSILON = 1e-6f;
        public const float RIGID_COMPLIANCE_THRESHOLD = 1e-9f;
        public const float MAX_LAMBDA_FRACTION = 0.5f;
        public const float MAX_CORRECTION_FRACTION = 0.1f;

        #endregion

        #region Node Rotation Constants
        public const int MAX_ROTATION_ITERATIONS = 5;
        public const float ROTATION_CONVERGENCE_EPSILON = 1e-9f;
        public const float MIN_OMEGA_MAGNITUDE = 1e-6f;

        #endregion

        #region Pressure Constants
        public const float PRESSURE_SCALE_FACTOR = 10f;
        public const float PRESSURE_DISTANCE_OFFSET = 0.1f;
        public const float MIN_CENTER_DISTANCE = 0.001f;

        #endregion

        #region Node Management Constants
        public const float MIN_NODE_RADIUS = 0.001f;
        public const float MAX_NODE_RADIUS = 1f;
        public const float DEFAULT_NODE_RADIUS = 0.01f;
        public const float DEFAULT_NODE_MASS = 0.5f;
        public const float MIN_TOTAL_MASS = 1e-6f;

        #endregion

        #region Debugging Constants
        public const float DEBUG_CROSS_SIZE = 0.05f;
        public const float DEBUG_NORMAL_LENGTH = 0.2f;
        public const float DEBUG_CORRECTION_SCALE = 5f;

        #endregion

        #region Job System Constants
        public const int VERLET_JOB_BATCH_SIZE = 64;

        #endregion

        #region Default Physics Values
        public const float DEFAULT_RESTITUTION = 0.02f;
        public const float DEFAULT_DYNAMIC_FRICTION = 0.6f;
        public const float DEFAULT_STATIC_FRICTION = 0.8f;
        public const float DEFAULT_COLLISION_DAMPING = 0.95f;
        public const float STANDARD_GRAVITY = 9.81f;
        public const float GRAVITY_TIMESTEP_SCALE = STANDARD_GRAVITY;

        #endregion

        #region Bounds and Distance Constants
        public const float BOUNDS_EXPANSION = 0.5f;
        public const float CUBE_CONNECTION_DISTANCE_MULTIPLIER = 1.5f;

        #endregion

        #region Friction Constants
        public const float MIN_TANGENT_SPEED = 0.0001f;

        #endregion
    }
}