import json

# Right now, this only processes the kraken log data
# outputs the epoch with the max confidence as well as the lat and long
# TODO: implement the fusion data

def log_generator(file_path):
    with open(file_path, 'r') as f:
        for line in f:
            yield json.loads(line)

# log file path; modify ts to py's file path location
file_path = '/content/doa_log.jsonl'

# track max confidence
max_record = None

try:
    for record in log_generator(file_path):
        if max_record is None or record.get('confidence', 0) > max_record.get('confidence', 0):
            max_record = record

    if max_record:
        epoch = max_record.get('epoch')
        conf = max_record.get('confidence')
        lat = max_record.get('latitude')
        lon = max_record.get('longitude')
        print(f"At epoch {epoch}, Highest confidence: {conf}, latitude: {lat}, longitude: {lon}")
    else:
        print("No records found in the log file.")
except FileNotFoundError:
    print(f"Error: The file {file_path} was not found.")
except Exception as e:
    print(f"An error occurred: {e}")

