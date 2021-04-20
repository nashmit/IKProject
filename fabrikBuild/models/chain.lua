meshes = {
  Base = {
    name = "Base",
    scale = { 0.05, 0.05, 0.05},
    color = { 0, 0, 0},
    src = "meshes/unit_cube.obj",
  },
  Link1 = {
    name = "Link1",
    dimensions = { 0.05, 0.6, 0.1},
    translate = {0, 0.3, 0},
    color = { 0.8, 0, 0.1},
    src = "meshes/unit_cube.obj",
  },
  Link2 = {
    name = "Link2",
    dimensions = { 0.05, 0.4, 0.1},
    translate = {0, 0.2, 0},
    color = { 0.9, 0, 0.5},
    src = "meshes/unit_cube.obj",
  },
  Link3 = {
    name = "Link3",
    dimensions = { 0.05, 0.9, 0.1},
    translate = {0, 0.45, 0},
    color = { 1, 0.6, 0.8},
    src = "meshes/unit_cube.obj",
  },
  Cube = {
    name = "Cube",
    dimensions = { 0.1, 0.1, 0.1},
    translate = {0, 0.05, 0},
    color = { 0.1, 1, 0.8},
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
      name = "Base",
      parent = "ROOT",
      joint_frame = {
        r = { 0, 0, 1 },
      },
      visuals = {
        meshes.Base,
      },
    },
    {
      name = "link1",
      parent = "ROOT",
      joint_frame = {
        r = { 0, 0, 1 },
      },
      joint = {{1, 0, 0, 0, 0, 0}},
      visuals = {
        meshes.Link1,
      },
    },
    {
      name = "link2",
      parent = "link1",
      joint_frame = {
        r = { 0, 0.6, 0},
      },
      joint = {{1, 0, 0, 0, 0, 0} --
      },
      visuals = {
        meshes.Link2,
      },
    },
    {
      name = "link3",
      parent = "link2",
      joint_frame = {
        r = { 0, 0.4, 0 },
      },
      joint = {{1, 0, 0, 0, 0, 0} --
      },
      visuals = {
        meshes.Link3,
      },
    },
    {
      name = "cube",
      parent = "link3",
      joint_frame = {
        r = { 0, 0.9, 0 },
      },
      joint = {{0, 1, 0, 0, 0, 0} --
      },
      visuals = {
        meshes.Cube,
      },
    },
  }
}

return model
