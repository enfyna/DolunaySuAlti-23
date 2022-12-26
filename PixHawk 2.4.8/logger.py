import datetime
def basla():
    file =open("log.txt","a")
    file.write("\\\ GOREV BASLADI ///"+str(datetime.datetime.now())+"\n")
    file.close()
def yaz(message):
    file =open("log.txt","a")
    file.write(str(datetime.datetime.now()) + " " + str(message) + "\n")
    file.close()