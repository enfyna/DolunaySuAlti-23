# Uçuş Kontrol Kartı

Uçuş kontrol kartı üretilecek olan İHA/SİDA'larda kullanılan ;içinde işlemci ve ivmeölçer, gyro, pusula gibi temel sensörler bulunduran gelen verilere göre PID hesaplamaları yaparak aracın motorlarına gereken gücü verip dengesini ve hareketini sağlar.

    Örnek olarak Pixhawk, APM, Navio2 ve Mamba verilebilir.

- PixHawk
    -

    Piyasada bilinen bir marka olması ,kullanmayı planladığımız tüm giriş/çıkış portlarına sahip olması ,internette ayarlanması ve kullanımı hakkında yeterli kaynaklara sahip olması nedeni ile seçilmesine karar verildi. 

    - Özellikler
        -

        - 168 mHz CortexM4F CPU
        - ST Micro L3GD20H 16 bit gyroscope
        - ST Micro LSM303D 14 bit accelerometer / magnetometer
        - Invensense MPU 6000 3-axis accelerometer/gyroscope
        - MEAS MS5611 barometer
        - 8x PWM out
        - ...


---

# Mesaj Arayüzü

İki aygıtın ya da yazılımın kendi aralarında gönderdikleri verileri anlamlandırmak için kullanılan protokollerdir.

- ArduPilot'un desteklediği Mavlink protokolünün kullanılmasına karar verildi.

    - Mavlink
        -

        Mavlink İHA/SİDA'larda kullanılan bir mesaj protokolüdür.Araçta Pixhawk ve JN arasındaki iletişimi 
sağlaması için kullanıldı.Python ile kullanabilmek için pymavlink kütüphanesinin kullanılması gerekmektedir.Yazılan kodlar JN'da çalıştırılır ve bu sayede pixhawk ile bağlantı kurulmuş olur.



        - Mavlink Kodlama (pymavlink)
            -
    
            Mavlink JN ve pixhawk arasında bağlantı kurulduktan sonra komutlar ve mesajlar kullanarak 2 sistem arasındaki iletişimi sağlar.
            Pixhawk'tan istediğimiz veriyi ya da yaptırmak istediğimiz görevin mesajını komutlar sayesinde göndeririz.Pixhawk gerekli işlemleri yaptıktan sonra yaptığı işlemin başarılı olup olmadığını gösteren bir mesajı "CMD_ACK" komutuyla geri döndürür.
            Bu komutun sonucuna göre bir sonraki adıma geçip geçemeyeceğimizi öğreniriz.
    

            - Örnek : 
                -
    
                    from pymavlink import mavutil #Kütüphaneyi ekle.

                    master = mavutil.mavlink_connection("/dev/ttyACM0")
                    #Bağlantı objesi oluştur.
                    #mavlink_connection fonksiyonuna pixhawkın
                    bulunduğu port yazılır.
                    #Port bilgisi doğru ise Pixhawk'a bağlanırız.

                    master.wait_heartbeat()
                    #Kalp atışını bekle
                    #Kalp atışları mavlinkin kullandığı bir mesaj tipidir.
                    #Bu mesajda Pixhawk belli aralıklarla Jetson
                    Nano'ya hangi modu kullandığı gibi temel bilgileri
                    gönderir.
                    
                    master.mav.command_long_send(master.target_system,master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, #Gönderilen komut
                    0, #Confirmation parametresi
                    1,0,0,0,0,0,0) #Gönderilen komutun parametreleri
                    #Yukarıdaki fonksiyon ile pixhawk'a komut gönderebiliyoruz.
                    #Bu örnekte arm etme komutunu gönderiyoruz.
                    #Arm etme komutunun 1. parametresi 1 olursa pixhawk'ı arm ,0 ise disarm eder.Bu örnekte 1 olduğu için pixhawk arm olacak.

                    msg2 = link.recv_match(type='COMMAND_ACK', blocking=True).to_dict() #CMD_ACK mesajını yakala.
                    print(msg2['result']) # Son verilen komutun sonucuna bak.Eğer 0 ise komut başarı ile gerçekleşmiştir.
    
   

---

# Yer Kontrol İstasyonu

Yer kontrol istasyonları üretilecek olan İHA/SİDA'ların kurulumunda kullanılan ve araç çalışırken araç ile iletişim haline geçen ,araçtan gelen bilgileri gösteren yazılımdır.

    Bu yazılımlara örnek olarak QGroundControl,Mission Planner,APM Planner,MAVProxy,Visionair ve UgCS verilebilir.


- QGroundControl (QGC)
    -
    Arayüzünün basit ,sade ve istenilen ayarların hızlıca bulunabilmesinden dolayı QGC'nin kullanılmasına karar verildi.

---

# Ana Bilgisayar

Aracın makine öğrenmesi, renk algılama, sensörlerden gelen bilgileri alma ve hesaplama gibi işlemleri yapması için bir bilgisayar gereklidir.

- Nvidia Jetson Nano (JN)
    -

    Nvidia Jetson Nano sahip olduğu işlem gücü ve küçük boyutu nedeni ile kullanılmasına karar verildi.

    - Özellikleri

        - Ekran Kartı	 : 128-core Maxwell
        - İşlemci 	 : Quad-core ARM A57 @ 1.43 GHz
        - Hafıza       : 4 GB 64-bit LPDDR4 25.6 GB/s
        - Boyut        : 100 mm x 80 mm x 29 mm
        - ...


---

# Sensörler

Sensörler istenilen bir fiziksel değişkeni algılayıp çıkış sinyali oluşturan cihazlardır.Araç dışarıdan destek almadan otonom bir şekilde çalışabilmesi için gerekli verileri sensörler sayesinde alır.

- Kamera
    -   
    Kamera görüntü almayı sağlayan sensördür.Logitech WebCam kullanılmasına karar verildi.
    
    - Özellikler
        - 1080p
        - USB bağlantı
        
- Hidrofon
    -       
    Hidrofon sudaki ses dalgalarının yakalamaya yarayan sensördür.

    - Özellikler
        - ...

- Mesafe sensörü
    -   
    Mesafe sensörü ses dalgaları göndererek yansıyan sesin arada geçen süresiyle mesafeyi hesaplayan sensördür.  
    - Özellikler
        - ...

- Basınç sensörü
    -       
    Basınç sensörü sudaki basıncı ölçerek aracın su yüzeyinin ne kadar altında olduğunu metre cinsinden veren sensörüdür.
    - Özellikler
        - ...

---

# Görevler

- 1.Görev : Renk tespiti ve konumlanma görevi
    ---
    Adımlar: 

        - Ara 
        - Bul
        - Kon



