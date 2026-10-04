# Continue the candidate adaptive solve in AEDT with up to 24 passes.
# Run through Tools > Run Script in the open AEDT Student session.
import os
import traceback
import ScriptEnv
ScriptEnv.Initialize("Ansoft.ElectronicsDesktop")
import __main__
oDesktop = getattr(__main__, "oDesktop", globals().get("oDesktop"))

# Folder where logs and CSV exports are written. Change this to your own folder.
ROOT = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
# Name of the open AEDT project created by build_4x4_FR4_JLC.py (change to match yours).
PROJECT = "FR4_5850_4x4_BandCandidate_20260929_201533"
DESIGN = "Array4x4_FR4_BandCandidate"

with open(os.path.join(ROOT, "band_candidate_refine_log.txt"), "w") as log:
    try:
        project = oDesktop.SetActiveProject(PROJECT)
        design = project.SetActiveDesign(DESIGN)
        setup = design.GetModule("AnalysisSetup")
        setup.EditSetup("Setup1", [
            "NAME:Setup1", "SolveType:=", "Single",
            "Frequency:=", "5.85GHz", "MaxDeltaS:=", 0.02,
            "PortsOnly:=", False, "UseMatrixConv:=", False,
            "MaximumPasses:=", 24, "MinimumPasses:=", 3,
            "MinimumConvergedPasses:=", 2, "PercentRefinement:=", 8,
            "IsEnabled:=", True, "BasisOrder:=", 1, "DoLambdaRefine:=", True,
            "DoMaterialLambda:=", True, "SetLambdaTarget:=", False,
            "PortAccuracy:=", 2, "SaveAnyFields:=", True,
            "SaveRadFieldsOnly:=", False])
        project.Save()
        log.write("Setup extended to 24 adaptive passes.\n")
        log.flush()
        design.AnalyzeAll()
        log.write("AnalyzeAll completed.\n")
        log.flush()
        reports = design.GetModule("ReportSetup")
        for name in ("S11_Candidate_Wide", "S11_ResonanceCheck_dB",
                     "AntennaParameters", "RealizedGain_Cuts_5p85GHz",
                     "RealizedGain_Eplane_Band"):
            reports.ExportToFile(name, os.path.join(ROOT, name + "_refined.csv"))
            log.write("Exported " + name + "\n")
            log.flush()
        project.Save()
        log.write("Project saved.\n")
    except Exception:
        log.write(traceback.format_exc())
        raise
