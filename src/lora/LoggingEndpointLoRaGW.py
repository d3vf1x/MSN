from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

from chirpstack_api import integration
from google.protobuf.json_format import Parse
import json
import csv
import os
from datetime import datetime

csv_file = "sensor_data.csv"
def process_uplink(body):
    try: 
        data = json.loads(body.decode('utf8').replace("'", '"'))
        #data = 

        # Extract required fields
        voltage = data["object"]["voltage"]
        humidity = data["object"]["humidity"]
        temperature = data["object"]["temperature"]
        seq_number = data["fCnt"]
        time_str = data["time"]
        time_stamp = '{:%Y-%m-%d %H:%M:%S}'.format(datetime.now())

        # Format the extracted data
        extracted_data = [time_stamp, time_str, seq_number, voltage, humidity, temperature]

        # Check if file exists; if not, write headers
        file_exists = os.path.isfile(csv_file)

        # Append data to the CSV file
        with open(csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                # Write header if file is created for the first time
                writer.writerow(["Time", "TimeGW", "SeqNumber", "Voltage", "Humidity", "Temperature"])
            writer.writerow(extracted_data)

        print(f"Data saved: {extracted_data}")

    except Exception as e:
        print(f"Error processing message: {e}")


class Handler(BaseHTTPRequestHandler):
    # True -  JSON marshaler
    # False - Protobuf marshaler (binary)
    json = False

    def do_POST(self):
        self.send_response(200)
        self.end_headers()
        query_args = parse_qs(urlparse(self.path).query)

        content_len = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_len)

        if query_args["event"][0] == "up":
            self.up(body)

        elif query_args["event"][0] == "join":
            self.join(body)

        else:
            print("handler for event %s is not implemented" % query_args["event"][0])

    def up(self, body):
        print("Uplink received", body)
        process_uplink(body)
        #up = self.unmarshal(body, integration.UplinkEvent())
        #print("Uplink received from: %s with payload: %s" % (up.device_info.dev_eui, up.data.hex()))

    def join(self, body):
        print("Join received", body)
        #join = self.unmarshal(body, integration.JoinEvent())
        #print("Device: %s joined with DevAddr: %s" % (join.device_info.dev_eui, join.dev_addr))

    def unmarshal(self, body, pl):
        if self.json:
            return Parse(body, pl)
        
        pl.ParseFromString(body)
        return pl

httpd = HTTPServer(('', 8090), Handler)
httpd.serve_forever()
