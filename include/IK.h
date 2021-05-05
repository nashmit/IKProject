//
// Created by nash_mit on 23.04.2021.
//

#ifndef KUKA_RBDL_IK_H
#define KUKA_RBDL_IK_H


#include "IKProject/config.h"

#ifndef RBDL_BUILD_ADDON_LUAMODEL
#error "Error: RBDL addon LuaModel not enabled."
#endif
#include <rbdl/addons/luamodel/luamodel.h>


#include "IKProject/Utils.h"

#include "IKProject/D_H_Parameterization.h"

#include "IKProject/HierarchyOfDHParameterization.h"

#include "IKProject/KuKa_KR5_R850_D_H.h"

#include "IKProject/Trihedral.h"

#endif //KUKA_RBDL_IK_H
