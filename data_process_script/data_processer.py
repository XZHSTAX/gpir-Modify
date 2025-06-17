#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import os

excel_file_path = "rosrecord/Exp1/Exp1-main/result/Exp1_D1_2025-06-17-22-00-20.xlsx"

ego_vehicle_status = pd.read_excel(excel_file_path, sheet_name="ego_vehicle_status")

a_lon_max = ego_vehicle_status['a_lon'].abs().max()
a_lat_max = ego_vehicle_status['a_lat'].abs().max()
dot_yaw_max = ego_vehicle_status['dot_yaw'].abs().max()
dot_yaw_ave = ego_vehicle_status['dot_yaw'].abs().mean()

print(a_lon_max,a_lat_max,dot_yaw_max,dot_yaw_ave)

