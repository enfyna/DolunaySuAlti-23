---
marp : true
---

# Uçuş Kontrol Kartı

Uçuş kontrol kartı üretilecek olan İHA/SİDA'larda kullanılan ;içinde işlemci ve ivmeölçer, gyro, pusula gibi temel sensörler bulunduran gelen verilere göre PID hesaplamaları yaparak aracın motorlarına gereken gücü verip dengesini ve hareketini sağlar.

    Örnek olarak Pixhawk, APM, Navio2 ve Mamba verilebilir.

---

PixHawk
---

Piyasada bilinen bir marka olması ,kullanmayı planladığımız tüm giriş/çıkış portlarına sahip olması ,internette ayarlanması ve kullanımı hakkında bir sürü kaynak olması nedeni ile seçilmesine karar verildi. 

- Özellikleri

        - 168 mHz CortexM4F CPU
        - ST Micro L3GD20H 16 bit gyroscope
        - ST Micro LSM303D 14 bit accelerometer / magnetometer
        - Invensense MPU 6000 3-axis accelerometer/gyroscope
        - MEAS MS5611 barometer
        - 8x PWM out
        - ...(Düzenlenmesi lazım)


---

# Yer Kontrol İstasyonu

Yer kontrol istasyonları üretilecek olan İHA/SİDA'ların kurulumunda kullanılan ve araç çalışırken araç ile iletişim haline geçen ,araçtan gelen bilgileri gösteren yazılımdır.

    Bu yazılımlara örnek olarak QGroundControl,Mission Planner,APM Planner,MAVProxy,Visionair ve UgCS verilebilir.


QGroundControl (QGC)
---
* Arayüzünün basit ,sade ve istenilen ayarların hızlıca bulunabilmesinden dolayı QGC'nin kullanılmasına karar verildi.

---

# Mesaj Arayüzü

İki aygıtın ya da yazılımın kendi aralarında gönderdikleri verileri anlamlandırmak için kullanılan protokollerdir.

    ArduPilot'un desteklediği Mavlink protokolünün kullanılmasına karar verildi.

Mavlink
---

Mavlink Pixhawk ve Jetson Nano arasındaki iletişimi 
sağlayacak olan protokoldür.Python ile kullanabilmek için pymavlink kütüphanesinin kullanılması gerekmektedir.Yazılan kodlar Jetson Nano'da çalıştırılır ve bu sayede bağlantı kurulmuş olur.

---

- Kodlama
    ---
    Kod örneği : 
    
        from pymavlink import mavutil #Kütüphane eklenir.

        master = mavutil.mavlink_connection("/dev/ttyACM0")
        #Bağlantı objesi olusturulur.
        #connection fonksiyonuna pixhawkın bulunduğu port yazılır.
        #Port bilgisi doğru ise Pixhawk'a bağlanırız.

        master.wait_heartbeat()
        #Kalp atışı bekle
        #Kalp atışlari mavlinkin kullandığı bir mesaj tipidir.
        #Bu mesajda Pixhawk belli aralıklarla Jetson Nano'ya hangi modu kullandığı gibi temel bilgileri gönderir.

        master.mav.command_long_send(master.target_system,master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, #Gönderilen komut
        0, #Confirmation parametresi
        1,0,0,0,0,0,0) #Gönderilen komutun parametreleri
        #Yukarıdaki fonksiyon ile pixhawk'a komut gönderebiliyoruz.
        #Bu örnekte arm etme komutunu gönderiyoruz.
        #Arm etme komutunun 1. parametresi 1 olursa pixhawk'ı arm 0 ise disarm eder.Bu örnekte 1 olduğu için pixhawk arm olacak.


---


# Nvidia Jetson Nano

Aracın makine öğrenmesi, renk algılama, sensörlerden gelen bilgileri alma ve hesaplama gibi işlemleri yapması için bir bilgisayar gereklidir.Nvidia Jetson Nano sahip olduğu işlem gücü ve küçük boyutu nedeni ile kullanılmasına karar verildi.

- Özellikleri

        - Ekran Kartı	    128-core Maxwell
        - İşlemci 	    Quad-core ARM A57 @ 1.43 GHz
        - Hafıza 	    4 GB 64-bit LPDDR4 25.6 GB/s
        - Boyut           100 mm x 80 mm x 29 mm




---

# Görev 1

