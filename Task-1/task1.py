"""
Cezeri Autonomous Flight Controller Task-1

HezarFen Team
Code Authors: Adil Sevim, Utku Özcan, Melek Serkaya, Cansu Melek Karpuz
"""

HEDEF_YAKINLIK = 0.5
PI = 3.141

class Cezeri(CezeriParent):
    """
    Autonomous agent controller.
    - Climb to a safe altitude, then navigate toward the active target.
    - Obstacle avoidance with local grid sectors.
    - Battery-aware diversion to the nearest charging station (for id=2).
    - Emergency diversion to the nearest hospital when `self.acil_durum` is True.
    - Landing logic when close to target.
    NOTE: Method/variable names are preserved; only comments and print messages are translated.
    """

    def __init__(self, id=1):
        # Correct constructor name must be __init__!
        super().__init__(id=id, keyboard=True, sensor_mode=DUZELTILMIS)
        self.baslangic_bolgesi = self.harita.bolge(float(self.gnss.enlem), float(self.gnss.boylam))
        print("Charging stations:", self.harita.sarj_istasyonlari)

        # Flags
        self.hedefe_ulasildi = False  # When target is reached, start landing
        self.yukseldi = False         # Has the agent climbed to sufficient altitude?
        self.sarj_modu = False        # Charging mode (when battery is insufficient)
        self.orjinal_hedef = None     # Keep original target coordinates
        self.sarj_hedef = None        # Charging-station coordinates
        self.sarjkalkis = False
        self.sarjtamamlandi = False

    def run(self):
        # 1) Altitude gate
        if not self.yukseldi:
            if self.gnss.irtifa < 95:
                self.yukari_git(HIZLI)
                return
            else:
                self.yukseldi = True

        # 2) If landing mode is active (target reached), keep descending
        if self.hedefe_ulasildi:
            print("Target reached, descending...")
            self.asagi_git(YAVAS)
            return

        # 3) Default target: original one at self.hedefler[0]
        hedef = self.hedefler[0]
        original_target_x = float(hedef.bolge.enlem)
        original_target_y = float(hedef.bolge.boylam)
        target_x, target_y = original_target_x, original_target_y

        # 4) Battery and emergency checks (for cezeri id=2, battery handling)
        if self.id == 2:
            if self.batarya.veri < self.minimum_gereken_batarya(original_target_x, original_target_y):
                if not self.sarj_modu:
                    self.orjinal_hedef = (original_target_x, original_target_y)
                    self.sarj_modu = True
                    self.sarj_hedef = self.en_yakin_sarj_istasyonu()
                    print("Insufficient battery, diverting to nearest charging station:", self.sarj_hedef)
                target_x, target_y = self.sarj_hedef

        # 5) Emergency: divert to the nearest hospital
        if self.acil_durum:
            print("EMERGENCY DETECTED, DIVERTING TO HOSPITAL...")
            mevcut_x = float(self.gnss.enlem)
            mevcut_y = float(self.gnss.boylam)
            en_yakin_uzaklik = float("inf")
            en_yakin_hastane = None

            # Find nearest hospital by Euclidean distance
            for hastane in self.harita.hastaneler:
                hastane_x = float(hastane.enlem)
                hastane_y = float(hastane.boylam)
                uzaklik = ((hastane_x - mevcut_x)**2 + (hastane_y - mevcut_y)**2)**0.5
                if uzaklik < en_yakin_uzaklik:
                    en_yakin_uzaklik = uzaklik
                    en_yakin_hastane = hastane

            if en_yakin_hastane is not None:
                target_x = float(en_yakin_hastane.enlem)
                target_y = float(en_yakin_hastane.boylam)
                print("Nearest hospital coordinates:", target_x, target_y)

        # 6) Compute steering toward current target
        mevcut_x = float(self.gnss.enlem)
        mevcut_y = float(self.gnss.boylam)

        hedef_acisi = self.aci_hesapla(target_x, target_y)
        mevcut_aci = self.yon_acisi()
        donme_derece = hedef_acisi - mevcut_aci
        if donme_derece > 180:
            donme_derece -= 360
        elif donme_derece < -180:
            donme_derece += 360
        self.don(self.derece_to_radyan(donme_derece))

        # 7) Local obstacle check: pick a free direction if blocked or no-fly
        gidecek_kare_idx = self.get_neighbor_index(self.yon_acisi())
        if self.yerel_harita[gidecek_kare_idx].engel or self.yerel_harita[gidecek_kare_idx].ucusa_yasakli_bolge:
            yeni_aci = self.get_free_direction(self.yon_acisi())
            aci_farki = yeni_aci - self.yon_acisi()
            if aci_farki > 180:
                aci_farki -= 360
            elif aci_farki < -180:
                aci_farki += 360
            self.don(self.derece_to_radyan(aci_farki))

        # 8) Forward motion (speed depends on id)
        if self.id == 3:
            self.ileri_git(ORTA)
        else:
            self.ileri_git(HIZLI)

        # 9) Arrival check and landing/charging handling
        print("Charging mode:", self.sarj_modu)
        if self.distance_to_target(target_x, target_y) < HEDEF_YAKINLIK:
            if self.id == 2 and self.sarj_modu is True:
                self.hedefe_ulasildi = True

                # If we reached the charging point but altitude is still high, keep descending
                if self.gnss.irtifa > 29.75:
                    print("Arrived at charging point, but altitude is high. Continuing to descend... (altitude:", self.gnss.irtifa, ")")
                    self.asagi_git(YAVAS)
                else:
                    print("Reached charging station/hospital, initiating landing and charging...")
                    self.dur()
                    if self.batarya.veri == 100:
                        self.sarj_yap()  # Charging simulation (sets battery to 100%)
                        # Reset flags after charging
                        self.sarj_modu = False
                        self.sarjtamamlandi = True
                        self.hedefe_ulasildi = False
                        self.yukseldi = False  # Ensures a climb on the next cycle
                        print("Charging complete, taking off again...")
                return
            else:
                self.hedefe_ulasildi = True
                print("Target reached, initiating landing...")
                self.dur()
                return

    def sarj_yap(self):
        print("Charging...")
        self.batarya.veri = 100
        print("Charging completed. Battery:", self.batarya.veri)

    def aci_hesapla(self, hedefx, hedefy):
        """
        Compute desired heading (degrees) from current GNSS to (hedefx, hedefy).
        Uses a Taylor-based atan approximation; returns [0, 360).
        """
        mevcutx = float(self.gnss.enlem)
        mevcuty = float(self.gnss.boylam)
        delta_x = hedefx - mevcutx
        delta_y = hedefy - mevcuty
        if delta_x == 0:
            return 90 if delta_y > 0 else 270
        tan_degeri = delta_y / delta_x
        derece = self.atan_taylor(tan_degeri)
        if delta_x < 0:
            derece += 180
        return derece % 360

    def minimum_gereken_batarya(self, hedefx, hedefy):
        """
        Estimate minimum required battery to reach (hedefx, hedefy).
        Linear function of Euclidean distance with a +20 buffer.
        """
        mevcut_x = float(self.gnss.enlem)
        mevcut_y = float(self.gnss.boylam)
        dx = hedefx - mevcut_x
        dy = hedefy - mevcut_y
        mesafe = self.sqrt_custom(dx * dx + dy * dy)
        return mesafe * 1.0 + 20

    def en_yakin_sarj_istasyonu(self):
        """
        Find nearest charging station based on current GNSS.
        Returns (x, y) tuple.
        """
        mevcut_x = float(self.gnss.enlem)
        mevcut_y = float(self.gnss.boylam)
        en_yakin_mesafe = float("inf")
        en_yakin_istasyon = None
        for istasyon in self.harita.sarj_istasyonlari:
            istasyon_x = float(istasyon.enlem)
            istasyon_y = float(istasyon.boylam)
            mesafe = ((mevcut_x - istasyon_x) ** 2 + (mevcut_y - istasyon_y) ** 2) ** 0.5
            if mesafe < en_yakin_mesafe:
                en_yakin_mesafe = mesafe
                en_yakin_istasyon = (istasyon_x, istasyon_y)
        return en_yakin_istasyon

    def atan_taylor(self, x):
        """
        Fast atan(x) degree approximation.
        For |x| > 1, uses a reciprocal transform and recursion.
        """
        if abs(x) > 1:
            return 90 - self.atan_taylor(1 / x) if x > 0 else -90 - self.atan_taylor(1 / x)
        x2 = x * x
        return x * (57.2958 - (x2 * (18.78 - (x2 * 6.4))))

    def derece_to_radyan(self, derece):
        """Convert degrees to radians using PI constant above."""
        return derece * (PI / 180)

    def yon_acisi(self):
        """Return heading in degrees, derived from magnetometer reading."""
        return float(self.manyetometre.veri) * 57.2958

    def distance_to_target(self, hedefx, hedefy):
        """Euclidean distance between current GNSS and target (hedefx, hedefy)."""
        mevcutx = float(self.gnss.enlem)
        mevcuty = float(self.gnss.boylam)
        return self.sqrt_custom((hedefx - mevcutx) ** 2 + (hedefy - mevcuty) ** 2)

    def sqrt_custom(self, x):
        """Newton–Raphson square root for positive x (10 iterations)."""
        if x == 0:
            return 0
        guess = x
        for _ in range(10):
            guess = 0.5 * (guess + x / guess)
        return guess

    def get_neighbor_index(self, heading):
        """
        Map a heading (deg) to an 8-neighbor sector index in `yerel_harita`.
        The mapping is environment-specific and preserved as-is.
        """
        sector = int((heading + 22.5) // 45) % 8
        mapping = {0: 5, 1: 2, 2: 1, 3: 0, 4: 3, 5: 6, 6: 7, 7: 8}
        return mapping[sector]

    def get_free_direction(self, current_heading):
        """
        Probe candidate headings around `current_heading` to find a free neighbor cell.
        Returns the chosen heading (deg). If none found, keep current heading.
        """
        offsets = [-45, 45, -90, 90, -135, 135]
        for offset in offsets:
            candidate = (current_heading + offset) % 360
            neighbor_idx = self.get_neighbor_index(candidate)
            if not self.yerel_harita[neighbor_idx].engel:
                return candidate
        return current_heading


# Object creation and main loop (unchanged)
cezeri_1 = Cezeri(id=1)
cezeri_2 = Cezeri(id=2)
cezeri_3 = Cezeri(id=3)

while robot.is_ok():
    cezeri_1.run()
    cezeri_2.run()
    cezeri_3.run()
