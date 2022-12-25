import datetime

def yaz(message):
    file = open("log.txt","a")
    file.write(str(datetime.datetime.now()) +" "+ str(message) + "\n")
    file.close()
