meshes = {
  Goal = {
    name = "Goal",
    scale = { 0.05, 0.05, 0.05},
    color = { 1, 0, 0},
    translate = {0, 0.025, 0},
    src = "meshes/unit_cube.obj",
  },

}

model = {
  configuration = {
    axis_front = { 1, 0, 0 },
    axis_up    = { 0, 0, 1 },
    axis_right = { 0, -1, 0 },
    rotation_order = { 2, 1, 0},
  },

  frames = {
    {
      name = "goal",
      parent = "ROOT",
      joint_frame = {
        r = { -0.3, -0.2, 0.3 },
      },
      visuals = {
        meshes.Goal,
      },
    },
  }
}

return model
