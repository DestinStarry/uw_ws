"""Streaming CSV summary. No ROS/pandas dependency and no whole-file loading."""
import argparse
import csv
import json
import math
from pathlib import Path


def summarize(directory):
    path=Path(directory).expanduser()
    if path.is_file(): path=path.parent
    result={"session_dir":str(path.resolve()),"rows":0,"first_elapsed_s":None,"last_elapsed_s":None,
            "armed_rows":0,"depth_usable_rows":0,"imu_fresh_rows":0,"ranges":{}}
    for name in ("metadata","summary"):
        p=path/(name+".json")
        try: result[name]=json.loads(p.read_text(encoding="utf-8")) if p.is_file() else None
        except (OSError,ValueError) as exc:result[name]={"read_error":str(exc)}
    fields=["depth_m","depth_error_m","cpu_temperature_c","mcu_voltage_v","rpm_0","rpm_1","rpm_2","rpm_3", "cycle_dt_s","planner_ms","mcu_spi_errors","mcu_rx_reject"]
    with (path/"flight.csv").open(newline="",encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            result["rows"]+=1
            for key,out in (("armed","armed_rows"),("depth_log_usable","depth_usable_rows"),("imu_fresh","imu_fresh_rows")):
                result[out]+=row.get(key) in ("True","1")
            try: elapsed=float(row["elapsed_s"])
            except (ValueError,KeyError,TypeError): elapsed=None
            if result["first_elapsed_s"] is None: result["first_elapsed_s"]=elapsed
            result["last_elapsed_s"]=elapsed
            for field in fields:
                try: value=float(row.get(field,""))
                except (ValueError,TypeError): continue
                if not math.isfinite(value): continue
                current=result["ranges"].setdefault(field,{"min":value,"max":value})
                current["min"]=min(current["min"],value);current["max"]=max(current["max"],value)
    result["warning"]="Ranges include latest cached values; filter by receive age, validity and usable flags for detailed analysis. Model forces are not measured forces."
    if result["summary"] is None: result["completion"]="incomplete_or_still_recording"
    else: result["completion"]="closed" if result["summary"].get("clean_close") else "incomplete"
    return result


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session",help="session directory or its flight.csv")
    options=parser.parse_args(args)
    print(json.dumps(summarize(options.session),ensure_ascii=False,indent=2,allow_nan=False))


if __name__=="__main__":main()
