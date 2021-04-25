print("Simple 1 joint")

meshes = {

    Base = {
        name = "Base",
        --scale = { 0.05, 0.05, 0.05},
        dimensions = { 0.1, 0.05, 0.05},
        --translate = {0, -1.5, 0},
        rotate = { axis = {1., 0., 0.}, angle = 75},
        color = { 0.5, 0.1, 0.5 },
        src = "meshes/unit_cube.obj",
    },

    Link1 = {
        name = "Link1",
        dimensions = { 1, 1, 1},
        --translate = {0, 0, -1/2},
        color = { 0.8, 0, 0.1},
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

        { -- Base note is fixed with respect to world space, no degree of freedom as it dones't contain any joint
            name = "Base",
            parent = "ROOT",
            joint_frame = {
                r = { 0, 0, 0 },
            },
            visuals = {
                meshes.Base,
            },
        },

        {
            name = "link1",
            parent = "ROOT",
            joint_frame = {
                r = { 0, 0, 0 },
            },
            joint = {
                { 1, 0, 0, 0, 0, 0 }
            },
            visuals = {
                meshes.Link1,
            },
        }

    }
}

return model