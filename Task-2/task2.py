HEDEF_YAKINLIK = 4
PI = 3.141


def aci_hesapla(self, hedefx, hedefy):
    mevcutx = float(self.gnss.enlem)
    mevcuty = float(self.gnss.boylam)
    delta_x = hedefx - mevcutx
    delta_y = hedefy - mevcuty
    if delta_x == 0:
        return 90 if delta_y > 0 else 270
    tan_degeri = delta_y / delta_x
    derece = atan_taylor(tan_degeri)
    if delta_x < 0:
        derece += 180
    return derece % 360

def atan_taylor(x):
    if abs(x) > 1:
        return 90 - atan_taylor(1 / x) if x > 0 else -90 - atan_taylor(1 / x)
    x2 = x * x
    return x * (57.2958 - (x2 * (18.78 - (x2 * 6.4))))

def derece_to_radyan(derece):
    return derece * (PI / 180)

def yon_acisi(self):
    return float(self.manyetometre.veri) * 57.2958

def distance_to_target(self, hedefx, hedefy):
    mevcutx = float(self.gnss.enlem)
    mevcuty = float(self.gnss.boylam)
    return sqrt_custom((hedefx - mevcutx)**2 + (hedefy - mevcuty)**2)

def sqrt_custom(x):
    if x == 0:
        return 0
    guess = x
    for _ in range(10):
        guess = 0.5 * (guess + x / guess)
    return guess

def get_neighbor_index(self, heading):
    sector = int((heading + 22.5) // 45) % 8
    mapping = {0: 5, 1: 2, 2: 1, 3: 0, 4: 3, 5: 6, 6: 7, 7: 8}
    return mapping[sector]

def get_free_direction(self, current_heading, check_otoyol=False):
    offsets = [-45, 45, -90, 90, -135, 135]
    for offset in offsets:
        candidate = (current_heading + offset) % 360
        neighbor_idx = get_neighbor_index(self, candidate)
        neighbor = self.yerel_harita[neighbor_idx]
        if not neighbor.engel and not neighbor.ucusa_yasakli_bolge:
            return candidate
    return current_heading  

# Cezeri sınıfı
class Cezeri(CezeriParent):
    def __init__(self, id=1):
        super().__init__(id=id, keyboard=True, sensor_mode=DUZELTILMIS)
        self.baslangic_bolgesi = self.harita.bolge(float(self.gnss.enlem), float(self.gnss.boylam))
        self.hedefe_ulasildi = False
        self.yukseldi = False
        self.baslangic_x = self.gnss.enlem
        self.baslangic_y = self.gnss.boylam
        self.inis = False
        self.pistler = [3, 4, 5, 6]
        self.mevcut_hedef_idx = 0
        self.sarj_modu = False

    def pist_index(self, pist):
        d = {1:108, 2:120, 3:132, 4:144, 5:156, 6:168, 7:180, 8:192, 9:204, 10:216, 11:228, 12:240,
             14:252, 15:264, 16:276, 17:288, 18:300, 19:312, 20:324, 21:336, 22:348, 23:360, 24:372,
             25:384, 26:396, 27:408, 28:420, 29:432, 30:444, 31:456, 32:468}
        return d[pist] if pist in d else None

    def pistte_sarj_istasyonu_var_mi(self, pist_index):
        hedef = self.harita.otoyol_veri[pist_index]
        hedef_x = float(hedef.enlem)
        hedef_y = float(hedef.boylam)
        for sarj in self.harita.sarj_istasyonlari:
            sarj_x = float(sarj.enlem)
            sarj_y = float(sarj.boylam)
            if abs(sarj_x - hedef_x) < HEDEF_YAKINLIK and abs(sarj_y - hedef_y) < HEDEF_YAKINLIK:
                return True
        return False

    def run(self):
        super().run()

        if not self.yukseldi:
            if self.gnss.irtifa < 310:
                self.yukari_git(HIZLI)
                return
            else:
                self.yukseldi = True
                print("Yeterli irtifaya ulaşıldı, otoyol navigasyonuna başlanıyor...")

        if self.sarj_modu:
            mesafe = float(self.radar.mesafe)
            if mesafe > 30:
                self.asagi_git(HIZLI)
                return
            elif mesafe > 6:
                self.asagi_git(YAVAS)
                return
            else:
                if self.batarya.veri >= 100:
                    print("Batarya tam doldu, kalkış yapılıyor...")
                    self.sarj_modu = False
                    self.yukseldi = False
                    self.mevcut_hedef_idx += 1
                return

        mevcut_hedef_pist = self.pistler[self.mevcut_hedef_idx]
        pist_idx = self.pist_index(mevcut_hedef_pist)
        hedef = self.harita.otoyol_veri[pist_idx]
        target_x = float(hedef.enlem)
        target_y = float(hedef.boylam)

        mevcut_x = float(self.gnss.enlem)
        mevcut_y = float(self.gnss.boylam)

        if distance_to_target(self, target_x, target_y) < HEDEF_YAKINLIK:
            print(f"Pist {mevcut_hedef_pist}'e ulaşıldı!")
            if mevcut_hedef_pist == 6:
                self.hedefe_ulasildi = True
                mesafe = float(self.radar.mesafe)
                if mesafe > 10:
                    self.asagi_git(HIZLI)
                    return
                elif mesafe > 6:
                    self.asagi_git(YAVAS)
                    return
                else:
                    print("Son iniş tamamlandı, simülasyon bitiyor...")
                    return
            else:
                if self.pistte_sarj_istasyonu_var_mi(pist_idx):
                    print(f"Pist {mevcut_hedef_pist}'te şarj istasyonu tespit edildi, şarj moduna geçiliyor...")
                    self.sarj_modu = True
                else:
                    self.mevcut_hedef_idx += 1
            return

        for otoyol_araci in self.harita.arac_trafigi:
            aracin_x = float(otoyol_araci.x)
            aracin_y = float(otoyol_araci.y)
            mesafe1 = aracin_x - mevcut_x
            if 0 > mesafe1 and mesafe1 > -50:
                print("ARAC VAR")
                self.don(3)
                self.ileri_git(ORTA)
            if 0 < mesafe1 and mesafe1 < 50:
                print("ÖNDE ARAC VAR")
                self.ileri_git(YAVAS)

        heading = yon_acisi(self)
        gidecek_kare_idx = get_neighbor_index(self, heading)
        komsu_bolge = self.yerel_harita[gidecek_kare_idx]
        if komsu_bolge.ucusa_yasakli_bolge or komsu_bolge.engel:
            yeni_aci = get_free_direction(self, heading)
            aci_farki = yeni_aci - heading
            if aci_farki > 180:
                aci_farki -= 360
            elif aci_farki < -180:
                aci_farki += 360
            self.don(derece_to_radyan(aci_farki))
            self.ileri_git(HIZLI)
            return

        hedef_acisi = aci_hesapla(self, target_x, target_y)
        mevcut_aci = yon_acisi(self)
        donme_derece = hedef_acisi - mevcut_aci
        if donme_derece > 180:
            donme_derece -= 360
        elif donme_derece < -180:
            donme_derece += 360

        if abs(donme_derece) > 5:
            self.don(derece_to_radyan(donme_derece) * 0.5)
            self.ileri_git(HIZLI)
        else:
            self.don(derece_to_radyan(donme_derece))
            self.ileri_git(HIZLI)

# Kargo sınıfı
class Kargo(KargoParent):
    def __init__(self, id=3):
        super().__init__(id=id, keyboard=True, sensor_mode=DUZELTILMIS)
        self.baslangic_bolgesi = self.harita.bolge(float(self.gnss.enlem), float(self.gnss.boylam))
        self.hedefe_ulasildi = False
        self.yukseldi = False
        self.baslangic_x = self.gnss.enlem
        self.baslangic_y = self.gnss.boylam
        self.kargo_birakildi = None
        self.inis = False

    def run(self):
        super().run()
        if not self.yukseldi:
            if self.gnss.irtifa < 54:
                self.yukari_git(HIZLI)
            else:
                self.yukseldi = True
        
        if len(self.harita.teslimat_bolgeleri) > 0 and self.kargo_durumu == True:
            hedef = self.harita.teslimat_bolgeleri[0]
            target_x = float(hedef.enlem)
            target_y = float(hedef.boylam)
            target_z = float(hedef.yukselti)
        elif len(self.harita.teslimat_bolgeleri) == 0 or self.kargo_durumu == False:
            hedef = self.baslangic_x, self.baslangic_y
            target_x = float(hedef[0])
            target_y = float(hedef[1])

        if self.inis == False and len(self.harita.teslimat_bolgeleri) == 0 and target_x == hedef[0] and target_y == hedef[1] and distance_to_target(self, target_x, target_y) < HEDEF_YAKINLIK:
            self.asagi_git(YAVAS)

        if self.kargo_durumu == False and not (hedef[0] - HEDEF_YAKINLIK < self.gnss.enlem < hedef[0] + HEDEF_YAKINLIK) and not (hedef[1] - HEDEF_YAKINLIK < self.gnss.boylam < hedef[1] + HEDEF_YAKINLIK):
            if self.gnss.irtifa < 58:
                self.yukari_git(YAVAS)

        if self.hedefe_ulasildi and self.batarya.veri > 99 and self.kargo_durumu == True:
            self.hedefe_ulasildi = False
            return
        elif self.hedefe_ulasildi:
            print("Hedefe ulaşıldı, iniş yapılıyor...")
            self.asagi_git(YAVAS)
            return

        mevcut_x = float(self.gnss.enlem)
        mevcut_y = float(self.gnss.boylam)

        hedef_acisi = aci_hesapla(self, target_x, target_y)
        mevcut_aci = yon_acisi(self)
        donme_derece = hedef_acisi - mevcut_aci

        gidecek_kare_idx = get_neighbor_index(self, yon_acisi(self))
        if self.yerel_harita[gidecek_kare_idx].engel or self.yerel_harita[gidecek_kare_idx].ucusa_yasakli_bolge:
            yeni_aci = get_free_direction(self, yon_acisi(self))
            aci_farki = yeni_aci - yon_acisi(self)
            if aci_farki > 180:
                aci_farki -= 360
            elif aci_farki < -180:
                aci_farki += 360
            self.don(derece_to_radyan(aci_farki))

        if len(self.harita.teslimat_bolgeleri) > 0:
            if distance_to_target(self, target_x, target_y) < HEDEF_YAKINLIK and not (target_x == self.harita.teslimat_bolgeleri[0].enlem and target_y == self.harita.teslimat_bolgeleri[0].boylam):
                self.hedefe_ulasildi = True
                print("Hedefe ulaşıldı, inişe geçiliyor...")
        elif len(self.harita.teslimat_bolgeleri) == 0 and distance_to_target(self, target_x, target_y) < HEDEF_YAKINLIK:
            self.hedefe_ulasildi = True
            print("Hedefe ulaşıldı, inişe geçiliyor...")

        if distance_to_target(self, target_x, target_y) < HEDEF_YAKINLIK:
            if len(self.harita.teslimat_bolgeleri) > 0:
                if target_x == self.harita.teslimat_bolgeleri[0].enlem and target_y == self.harita.teslimat_bolgeleri[0].boylam:
                    if self.gnss.irtifa < target_z:
                        self.yukari_git(YAVAS)
                        self.teslim_et()
                    elif self.gnss.irtifa > target_z:
                        self.asagi_git(YAVAS)
                        self.teslim_et()
        else:
            if donme_derece > 180:
                donme_derece -= 360
            elif donme_derece < -180:
                donme_derece += 360
            self.don(derece_to_radyan(donme_derece))
            self.ileri_git(HIZLI)

# Itfaiye sınıfı
class Itfaiye(ItfaiyeParent):
    def __init__(self, id=2):
        super().__init__(id=id, keyboard=True, sensor_mode=DUZELTILMIS)
        self.baslangic_bolgesi = self.harita.bolge(float(self.gnss.enlem), float(self.gnss.boylam))
        self.hedefe_ulasildi = False
        self.yukseldi = False
        self.baslangic_x = self.gnss.enlem
        self.baslangic_y = self.gnss.boylam
        self.inis = False

    def run(self):
        super().run()
        if not self.yukseldi:
            if self.gnss.irtifa < 95:
                self.yukari_git(HIZLI)
                return
            else:
                self.yukseldi = True

        if len(self.harita.yangin_bolgeleri) > 0 and self.su_seviyesi > 0:
            hedef = self.harita.yangin_bolgeleri[0]
            target_x = float(hedef.enlem)
            target_y = float(hedef.boylam)
        elif len(self.harita.yangin_bolgeleri) == 0 or self.su_seviyesi == 0:
            hedef = self.baslangic_x, self.baslangic_y
            target_x = float(hedef[0])
            target_y = float(hedef[1])

        if self.su_seviyesi > 0 and self.hedefe_ulasildi and self.batarya.veri > 99 and not len(self.harita.yangin_bolgeleri) == 0:
            self.yukseldi = False
            self.hedefe_ulasildi = False
            return
        elif self.hedefe_ulasildi:
            print("Hedefe ulaşıldı, iniş yapılıyor...")
            self.asagi_git(YAVAS)
            return

        mevcut_x = float(self.gnss.enlem)
        mevcut_y = float(self.gnss.boylam)

        hedef_acisi = aci_hesapla(self, target_x, target_y)
        mevcut_aci = yon_acisi(self)
        donme_derece = hedef_acisi - mevcut_aci

        gidecek_kare_idx = get_neighbor_index(self, yon_acisi(self))
        if self.yerel_harita[gidecek_kare_idx].engel or self.yerel_harita[gidecek_kare_idx].ucusa_yasakli_bolge:
            yeni_aci = get_free_direction(self, yon_acisi(self))
            aci_farki = yeni_aci - yon_acisi(self)
            if aci_farki > 180:
                aci_farki -= 360
            elif aci_farki < -180:
                aci_farki += 360
            self.don(derece_to_radyan(aci_farki))

        if len(self.harita.yangin_bolgeleri) == 0 or self.su_seviyesi == 0:
            if distance_to_target(self, target_x, target_y) < HEDEF_YAKINLIK:
                self.hedefe_ulasildi = True
                print("Hedefe ulaşıldı, inişe geçiliyor...")

        if len(self.harita.yangin_bolgeleri) >= 0:
            if distance_to_target(self, target_x, target_y) > HEDEF_YAKINLIK:
                self.su_ac(False)

        if distance_to_target(self, target_x, target_y) < HEDEF_YAKINLIK:
            if len(self.harita.yangin_bolgeleri) > 0:
                if target_x == self.harita.yangin_bolgeleri[0].enlem and target_y == self.harita.yangin_bolgeleri[0].boylam:
                    self.su_ac(True)
        else:
            if donme_derece > 180:
                donme_derece -= 360
            elif donme_derece < -180:
                donme_derece += 360
            self.don(derece_to_radyan(donme_derece))
            self.ileri_git(HIZLI)

        if self.su_seviyesi == 0:
            self.su_ac(False)

# Ana program
cezeri_1 = Cezeri(id=1)
kargo_1 = Kargo(id=3)
itfaiye_1 = Itfaiye(id=2)

while robot.is_ok():
    cezeri_1.run()
    kargo_1.run()
    itfaiye_1.run()