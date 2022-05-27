import requests
import json
import time
from PIL import Image, ImageDraw
import mysql.connector

cnx = mysql.connector.connect(user='sql11489402', password='8Dl4twiWj3', host='sql11.freemysqlhosting.net', database='sql11489402')
cursor = cnx.cursor()
query = ("REPLACE INTO sensors (Sensor, Value) VALUES ('RP_1', %s)" % db_sum)
cursor.execute(query)
cursor.close()
cnx.close()

token = "5337762612:AAGjat08AI6OC349MBieS3NzfemzFBznk3M"
url = f"https://api.telegram.org/bot{token}"
offset_id = "0"
ans = None

while True:
    method = f"/getUpdates?offset={offset_id}"
    answer = requests.get(url + method)
    content = answer.content
    data = json.loads(content)
    if data["ok"] and len(data["result"]) > 0:
        offset_id = str(data["result"][0]["update_id"]+1)
        chat_id = data["result"][0]["message"]["chat"]["id"]
        text = data["result"][0]["message"]["text"]

        if text == "/2":
            cnx = mysql.connector.connect(user='sql11489402', password='8Dl4twiWj3',
                                          host='sql11.freemysqlhosting.net',
                                          database='sql11489402')
            cursor = cnx.cursor()

            query = ("SELECT * FROM sensors")
            cursor.execute(query)
            x = cursor.fetchall()

            social = {}

            for sensor in x:
                sn = sensor[0]
                query = ("SELECT * FROM config WHERE sensor='{}'".format(sensor))
                cursor.execute(query)
                y = cursor.fetchall()
                if sn == 'ESP32_1':
                    db_32 = sensor[1]
                    if db_32 >= y[2]:
                        loc_x = y[0]
                        loc_y = y[1]
                        social{"esp"} = (loc_x, loc_y)
                elif sn == 'RP_1':
                    db_rp = sensor[1]
                    if db_rp >= y[2]:
                        loc_x = y[0]
                        loc_y = y[1]
                        social{"rp"} = (loc_x, loc_y)
                else:
                    zone_1 = sensor[0]
                    if zone_1 >= y[2]:
                        loc_1_x = y[0]
                        loc_1_y = y[1]
                        social{"z1"} = (loc_1_x, loc_1_y)
                    zone_2 = sensor[1]
                    if zone_2 >= y[5]:
                        loc_2_x = y[3]
                        loc_2_y = y[4]
                        social{"z2"} = (loc_2_x, loc_2_y)
                    zone_3 = sensor[2]
                    if zone_3 >= y[8]:
                        loc_1_x = y[6]
                        loc_1_y = y[7]
                        social{"z3"} = (loc_3_x, loc_3_y)

            cursor.close()
            cnx.close()

            for key, value in social.items():
                with Image.open('chatbot/grundriss.jpg') as img:
                    draw = ImageDraw.Draw(img)
                    draw.ellipse((value[0], value[1], value[0]+50, value[1]+50), fill="red")
                    img.save("ans.jpg")

            ans = "You have the opportunity to socialize at any of the marked points:"
            params = {"chat_id": chat_id, "text": ans}
            method = "/sendMessage"
            requests.post(url+method, params=params)
            params = {"chat_id": chat_id}
            method = "/sendPhoto"
            with open("ans.jpg", "rb") as img:
                requests.post(url+method, params=params, files={"photo": img})

            ans = "Thanks for using my help. Please type any message to restart my service."
            params = {"chat_id": chat_id, "text": ans}
            method = "/sendMessage"
            requests.post(url+method, params=params)
        elif text == "/1":
            cnx = mysql.connector.connect(user='sql11489402', password='8Dl4twiWj3',
                                          host='sql11.freemysqlhosting.net',
                                          database='sql11489402')
            cursor = cnx.cursor()
            query = ("SELECT * FROM sensors")
            cursor.execute(query)
            x = cursor.fetchall()
            for sensor in x:
                sn = sensor[0]
                if sn == 'IP_Cam':
                    vals = sensor[1].split(",")
                    tot = vals[2]
            cursor.close()
            cnx.close()

            method = "/sendMessage"
            ans = "There are currently {} persons in the Square.".format(tot)
            params = {"chat_id": chat_id, "text": ans}
            requests.post(url+method, params=params)
            ans = "Thanks for using my help. Please type any message to restart my service."
            params = {"chat_id": chat_id, "text": ans}
            requests.post(url+method, params=params)
        else:
            method = "/sendMessage"
            ans = "Hi, I am Squary,\n\n the overseer, supervisor, and well-known spirit of the Square!\n\nI can inform you about the general occupancy and recommend locations to socialize.\n\nSo with my support, the Square can better fulfill its role as a place for dialogue and exchange of experiences."
            params = {"chat_id": chat_id, "text": ans}
            requests.post(url+method, params=params)
            ans = "Please select your service:\n\n/1 : How many people are currently in the Square?\n\n/2 : Where can I socialize?"
            params = {"chat_id": chat_id, "text": ans}
            requests.post(url+method, params=params)
