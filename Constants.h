/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */
#pragma once

namespace PhysicsConstants
{
    // Collision
    constexpr float MIN_COLLISION_DISTANCE = 0.0001f;
    constexpr float DEFAULT_COLLISION_EPSILON = 1e-6f;
    constexpr float DEFAULT_TRIANGLE_THICKNESS = 0.02f;
    constexpr float SKIN_WIDTH = 0.005f;

    // Constraint
    constexpr float MIN_BEAM_LENGTH = 0.01f;
    constexpr float MAX_ALLOWED_DISPLACEMENT = 1.0f;
    constexpr float CONSTRAINT_EPSILON = 1e-6f;
    constexpr float RIGID_COMPLIANCE_THRESHOLD = 1e-9f;
    constexpr float CROSS_BODY_CONSTRAINT_DAMPING = 0.0f;

    // Node rotation (Müller polar decomposition)
    constexpr int   MAX_ROTATION_ITERATIONS = 5;
    constexpr float ROTATION_CONVERGENCE_EPSILON = 1e-9f;
    constexpr float MIN_OMEGA_MAGNITUDE = 1e-6f;
    constexpr float MIN_TOTAL_MASS = 1e-6f;

    // Pressure
    constexpr float PRESSURE_SCALE_FACTOR = 10.0f;
    constexpr float MIN_CENTER_DISTANCE = 0.001f;

    // Default physics
    constexpr float DEFAULT_NODE_MASS = 0.5f;
    constexpr float DEFAULT_RESTITUTION = 0.02f;
    constexpr float DEFAULT_DYNAMIC_FRICTION = 0.6f;
    constexpr float DEFAULT_STATIC_FRICTION = 0.8f;
    constexpr float STANDARD_GRAVITY = 9.81f;
    constexpr float MIN_TANGENT_SPEED = 0.0001f;

    // Spatial-hash cell sizes (must match C# CollisionHandler constants)
    constexpr float SPATIAL_CELL_SIZE = 0.15f;
    constexpr float FACE_NODE_CELL_SIZE = 0.3f;
    constexpr float STATIC_GRID_CELL_SIZE = 2.0f;
}