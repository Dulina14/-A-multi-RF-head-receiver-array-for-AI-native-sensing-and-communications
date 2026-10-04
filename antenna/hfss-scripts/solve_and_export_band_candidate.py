# Run from AEDT Tools > Run Script after build_4x4_FR4_band_candidate.py.
# Solves the newly created project and exports numeric reports beside it.
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
LOG = os.path.join(ROOT, "band_candidate_solve_log.txt")

with open(LOG, "w") as log:
    try:
        project = oDesktop.SetActiveProject(PROJECT)
        design = project.SetActiveDesign(DESIGN)
        log.write("Solving " + PROJECT + "/" + DESIGN + "\n")
        log.flush()
        design.AnalyzeAll()
        log.write("AnalyzeAll completed.\n")
        log.flush()
        reports = design.GetModule("ReportSetup")
        try:
            reports.CreateReport(
                "S11_Candidate_Wide", "Modal Solution Data", "Rectangular Plot",
                "Setup1 : WideSweep", [], ["Freq:=", ["All"]],
                ["X Component:=", "Freq",
                 "Y Component:=", ["dB(S(P1,P1))"]], [])
            log.write("WideSweep S11 report created.\n")
        except Exception:
            log.write("WideSweep S11 report could not be created:\n" +
                      traceback.format_exc() + "\n")
        for name in ("S11_Candidate_Wide", "S11_ResonanceCheck_dB",
                     "AntennaParameters", "RealizedGain_Cuts_5p85GHz",
                     "RealizedGain_Eplane_Band"):
            try:
                reports.ExportToFile(name, os.path.join(ROOT, name + ".csv"))
                log.write("Exported " + name + "\n")
            except Exception:
                log.write("Export failed for " + name + ":\n" +
                          traceback.format_exc() + "\n")
        project.Save()
        log.write("Project saved.\n")
    except Exception:
        log.write("Solve failed:\n" + traceback.format_exc() + "\n")
        raise
