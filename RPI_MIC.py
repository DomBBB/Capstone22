import subprocess
import math
import mysql.connector

process = subprocess.Popen("/home/pi/.local/bin/soundmeter", stdout=subprocess.PIPE, stderr=None, shell=True)

DBs = []
count = -1
while process.poll() is None:
    line_buffer = []
    while True:
        c = process.stdout.read(1)
        line_buffer.append(c)
        if c == '\r':
            break
    val = ''.join(line_buffer)
    if val != '':
        vals = val.split()
        if vals != []:
            DBs.append(20*math.log(float(vals[0]), 10))
    if len(DBs)%10 == 0:
        if count >= 0:
            db_sum = sum(DBs[10*count:10*count+10]) // 10
            cnx = mysql.connector.connect(user='sql11489402', password='8Dl4twiWj3', host='sql11.freemysqlhosting.net', database='sql11489402')
            cursor = cnx.cursor()
            query = ("REPLACE INTO sensors (Sensor, Value) VALUES ('RP_1', %s)" % db_sum)
            cursor.execute(query)
            cursor.close()
            cnx.close()
        count += 1
