#############################################################################
#
# This file is part of the libPR software.
# Copyright (C) 2017 by MIS lab (UPJV). All rights reserved.
#
# See http://mis.u-picardie.fr/~g-caron/fr/index.php?page=7 for more information.
#
# This software was developed at:
# MIS - UPJV
# 33 rue Saint-Leu
# 80039 AMIENS CEDEX
# France
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# libPR configuration file. Freely inspired from the 
# ViSP library. 
#
# Authors:
# Guillaume Caron
#
#############################################################################


pr_add_module(acquisition_representation per_core per_io)# OPTIONAL visp_blob visp_me)
pr_glob_module_sources()
pr_module_include_directories()
pr_create_module()
#pr_add_tests()


target_compile_definitions(per_acquisition_representation PUBLIC
    PER_SI_DIR="${PER_BINARY_DIR}/data/subdividedIcosahedron/"
)