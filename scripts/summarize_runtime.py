import csv, statistics
vals=[]
with open('results/device_runtime_measurements.csv', newline='') as f:
    for r in csv.DictReader(f): vals.append(int(r['response_ms']))
print('n =', len(vals))
print('mean_ms =', round(statistics.mean(vals),1))
print('median_ms =', statistics.median(vals))
print('min_ms =', min(vals))
print('max_ms =', max(vals))
