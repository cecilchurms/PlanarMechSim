# ********************************************************************************
# *                                                                              *
# *   This program is free software; you can redistribute it and/or modify       *
# *   it under the terms of the GNU Lesser General Public License (LGPL)         *
# *   as published by the Free Software Foundation; either version 3 of          *
# *   the License, or (at your option) any later version.                        *
# *   for detail see the LICENCE text file.                                      *
# *                                                                              *
# *   This program is distributed in the hope that it will be useful,            *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of             *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
# *   See the GNU Lesser General Public License for more details.                *
# *                                                                              *
# *   You should have received a copy of the GNU Lesser General Public           *
# *   License along with this program; if not, write to the Free Software        *
# *   Foundation, Inc., 59 Temple Place, Suite 330, Boston,                      *
# *   MA 02111-1307, USA                                                         *
# *_____________________________________________________________________________ *
# *                                                                              *
# *        ##########################################################            *
# *     #### PlanarMechSim - FreeCAD WorkBench - Revision 1.0 (c) 2025: ####     *
# *        ##########################################################            *
# *                                                                              *
# *               This program suite is an expansion of the                      *
# *                  "Nikra-DAP" workbench for FreeCAD                           *
# *                                                                              *
# *                         Software Development:                                *
# *                     Cecil Churms <churms@gmail.com>                          *
# *                                                                              *
# *             It is based on the MATLAB code Complementary to                  *
# *                  Chapters 7 and 8 of the textbook:                           *
# *                                                                              *
# *                     "PLANAR MULTIBODY DYNAMICS                               *
# *         Formulation, Programming with MATLAB, and Applications"              *
# *                          Second Edition                                      *
# *                         by P.E. Nikravesh                                    *
# *                          CRC Press, 2018                                     *
# *                                                                              *
# *     The original project (Nikra-DAP) was the vision of Lukas du Plessis      *
# *                      <lukas.duplessis@uct.ac.za>                             *
# *              who facilitated its development from the start                  *
# *                                                                              *
# *                     With the advent of FreeCAD 1.x,                          *
# *        the Nikra-DAP software was no longer compatible with the new,         *
# *                    built-in, Assembly functionality.                         *
# *          Nikra-DAP thus underwent a major re-write and was enlarged          *
# *                into the Mechanical Simulator: "PlanarMechSim"                *
# *                                                                              *
# *              The initial stages of  Nikra-DAP  were supported by:            *
# *                 Engineering X, an international collaboration                *
# *                founded by the Royal Academy of Engineering and               *
# *                        Lloyd's Register Foundation.                          *
# *                                                                              *
# *                   An early version of Nikra-DAP was written by:              *
# *            Alfred Bogaers (EX-MENTE) <alfred.bogaers@ex-mente.co.za>         *
# *                          with contributions from:                            *
# *                 Dewald Hattingh (UP) <u17082006@tuks.co.za>                  *
# *                 Varnu Govender (UP) <govender.v@tuks.co.za>                  *
# *                                                                              *
# *                          Copyright (c) 2025                                  *
# *_____________________________________________________________________________ *
# *                                                                              *
# *             Please refer to the Documentation and README for                 *
# *         more information regarding this WorkBench and its usage              *
# *                                                                              *
# ********************************************************************************
import FreeCAD as CAD
import FreeCADGui as CADGui
import SimClasses as SC
import SimViewProvider as SV

# =============================================================================
def makeSimGlobal(name="SimGlobal"):
    """Create a Sim initialisation FreeCAD group object"""

    simGlobalObject = CAD.ActiveDocument.addObject("App::DocumentObjectGroupPython", name)
    # Instantiate a SimGlobal object
    SC.SimGlobalClass(simGlobalObject)
    # Instantiate the class to handle the Gui stuff
    SV.ViewProviderSimGlobalClass(simGlobalObject.ViewObject)
    return simGlobalObject
# =============================================================================
def makeSimSolver(name="SimSolver"):
    """Create a Sim Solver object"""

    solverObject = CAD.ActiveDocument.addObject("Part::FeaturePython", name)
    # Instantiate a SimSolver object
    SC.SimSolverClass(solverObject)
    # Instantiate the class to handle the Gui stuff
    SV.ViewProviderSimSolverClass(solverObject.ViewObject)
    return solverObject
# =============================================================================
def makeSimMaterial(name="SimMaterial"):
    """Create a Sim Material object"""

    materialObject = CAD.ActiveDocument.addObject("Part::FeaturePython", name)
    # Instantiate a SimMaterial object
    SC.SimMaterialClass(materialObject)
    # Instantiate the class to handle the Gui stuff
    SV.ViewProviderSimMaterialClass(materialObject.ViewObject)
    return materialObject
# =============================================================================
def makeSimForce(name="SimForce"):
    """Create a Sim Force object"""

    forceObject = CAD.ActiveDocument.addObject("Part::FeaturePython", name)
    # Instantiate a SimForce object
    SC.SimForceClass(forceObject)
    # Instantiate the class to handle the Gui stuff
    SV.ViewProviderSimForceClass(forceObject.ViewObject)
    return forceObject
# =============================================================================
"""def makeSimBody(name="SimBody"):
    # Create an empty Sim Body object
    
    bodyObject = CAD.ActiveDocument.addObject("Part::FeaturePython", name)
    # Instantiate a SimBody object
    DC.SimBodyClass(bodyObject)
    # Instantiate the class to handle the Gui stuff
    SV.ViewProviderSimBodyClass(bodyObject.ViewObject)
    return bodyObject
    """
# =============================================================================
"""def makeSimJoint(name="SimJoint"):
    # Create an empty Sim Joint Object
    
    jointObject = CAD.ActiveDocument.addObject("Part::FeaturePython", name)
    # Instantiate a SimJoint object
    DC.SimJointClass(jointObject)
    # Instantiate the class to handle the Gui stuff
    SV.ViewProviderSimJointClass(jointObject.ViewObject)
    return jointObject
    """
# =============================================================================
