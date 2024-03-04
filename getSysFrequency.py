from datetime import datetime, timezone, timedelta
import requests
import matplotlib.pyplot as plt
import numpy as np
import os
import json

# Get current time and time-5 minutes
timeNow = datetime.now(timezone.utc)
timeNow = timeNow.isoformat(timespec='seconds')
timeNow = timeNow.split('+',1)[0]

timeBefore = datetime.now(timezone.utc) - timedelta(minutes=5)
timeBefore = timeBefore.isoformat(timespec='seconds')
timeBefore = timeBefore.split('+',1)[0]

# Specify format of requested data (json, xml, csv)
outputFormat = 'json'

# Create the url from which to make the get request, using above variables
url = "https://data.elexon.co.uk/bmrs/api/v1/datasets/FREQ?measurementDateTimeFrom={0}&measurementDateTimeTo={1}&format={2}".format(
    timeBefore, timeNow, outputFormat)

# Get data from url
response = requests.get(url)
data = response.json()
#print(data)

# Save data to disk
file_name = f'freq_{datetime.now().strftime("%y%m%dT%H%M%S_%f")}.json'
output_folder_path = 'pulled_json_data'
output_file_path = os.path.join(output_folder_path, file_name)

if not os.path.exists(output_folder_path):
    os.mkdir(output_folder_path)

with open(output_file_path, 'w') as f:
    json.dump(data, f)

## Plot data using matplotlib
# Reformat data from json into x list and y list
contents = data['data']
datetimestamps = [i['measurementTime'] for i in contents]
datetimestamps.reverse()
freqs = [i['frequency'] for i in contents]
freqs.reverse()

# Get just the time of the data, without the date
timestamps = [i.partition('T')[2].partition('Z')[0] for i in datetimestamps]
date = datetimestamps[1].partition('T')[0]

# Do the plotting
plt.plot(timestamps, freqs)
plt.title(f'Frequency data on date: {date}')
plt.xlabel('Time of Measurement')
plt.ylabel('Frequency (Hz)')
plt.gcf().autofmt_xdate()

plt.show()
