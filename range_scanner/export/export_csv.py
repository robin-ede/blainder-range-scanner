import csv
import numpy as np
import os

def export(filePath, fileName, data, exportNoiseData, rawData=None):
    print("Exporting data into .csv format...")

    with open(os.path.join(filePath, "%s.csv" % fileName), 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=";", quotechar='|', quoting=csv.QUOTE_MINIMAL)

        counts = {}
        if rawData is not None:
            for hit in rawData:
                s_id = hit.sensor_id if hit.sensor_id else "unknown"
                counts[s_id] = counts.get(s_id, 0) + 1
        
        print(f"Exporting {len(data)} hits. Sensor counts: {counts}")
        if rawData is not None and len(rawData) > 0:
            sample_ids = [h.sensor_id for h in rawData[:5]]
            print(f"Sample sensor IDs from rawData: {sample_ids}")

        if exportNoiseData:
            # write header to file
            writer.writerow(["sensor_id", "categoryID", "partID", "X", "Y", "Z", "distance", "X_noise", "Y_noise", "Z_noise", "distance_noise", "intensity", "red", "green", "blue"])

            for i, hit in enumerate(data):
                # get sensor_id from raw data if available
                sensor_id = rawData[i].sensor_id if rawData is not None else ""

                # write data to file
                writer.writerow(
                    [
                        sensor_id,
                        hit[0], hit[1],
                        hit[2], hit[3], hit[4],
                        hit[5],
                        hit[10], hit[11], hit[12],
                        hit[13],
                        hit[6],
                        hit[7], hit[8], hit[9]
                    ]
                )
        else:
            # write header to file
            writer.writerow(["sensor_id", "categoryID", "partID", "X", "Y", "Z", "distance", "intensity", "red", "green", "blue"])

            for i, hit in enumerate(data):
                # get sensor_id from raw data if available
                sensor_id = rawData[i].sensor_id if rawData is not None else ""

                writer.writerow(
                    [
                        sensor_id,
                        hit[0], hit[1],
                        hit[2], hit[3], hit[4],
                        hit[5],
                        hit[6],
                        hit[7], hit[8], hit[9]
                    ]
                )

    print("Done.")
