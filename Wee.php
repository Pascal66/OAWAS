<?php
if (!function_exists("Mysql_Connexion")) {include ("mainfile.php");}
// Utilisation de MySql
$table = 'meteo.wee';

// WIND
// U = -sin(direction) * wind_speed;
// V = -cos(direction) * wind_speed;
// Wind_speed = square_root(U*U + V*V);
// if V < 0 then Wind_direction = arctan(U/V) * 180/PI;
// Else Wind_direction = arctan(U/V) * 180/PI + 180;


if ($_REQUEST[SUN]) soleil(); //exit();
if ($_REQUEST[SOIL]) soil();  //exit();
//$solar = soleil();
if ($_REQUEST[WD]) {
//$solar = soleil(190);
    $result = sql_query("SELECT * FROM ".$table." ORDER BY TimeStamp DESC LIMIT 1;");
    list($Date, $Temperature, $Humidity, $Vcc, $A7, $solar) = sql_fetch_row($result);
    echo $Date." ".$Temperature." ".$Humidity." ".$solar." Analog: ".$A7;
} else {
$solar_coord = soleil($_REQUEST[A7]);

$solar_hauteur = $solar_coord[0];
$solar_azimuth = $solar_coord[1];
$solar_wm2     = $solar_coord[2];

$RF = $_REQUEST[RF];

$charge = "CHA";
if($_REQUEST[Vcc]<3283) $charge="DEC";
    $resultm = sql_query("INSERT INTO ".$table." VALUES (NULL, $_REQUEST[TEMP], $_REQUEST[HUM], $_REQUEST[Vcc], $_REQUEST[A7], '$solar_wm2', $_REQUEST[BARO]/100, '$charge',  $_REQUEST[RSSI], '$RF') ");
    echo "OK\r\n";


function soleil($nebu = 1024) {
$cloud = $nebu / 1024;
// on va dire que c'est l’indice de Perreaudau (In).
// Pour les conditions claires, In>0.8. Pour les conditions très maussades, In<0.1.
// http://www.cuepe.ch/html/biblio/pdf/baudry-ineichen%201996%20-%20rayonnement%20uvb%20a%20geneve.pdf

// re.jrc.ec.europa.eu/pvgis/solres/solmod3.htm
$clientrawfile = 'modules/meteo/clientraw.txt';
$dataraw = file_get_contents($clientrawfile);

      // clean up any blank lines
      $dataraw = trim($dataraw);
      $dataraw = preg_replace("/[\r\n]+[\s\t]*[\r\n]+/","\n",$dataraw);
      $data = explode(" ", $dataraw);

// TEMPERATURE
(float)$TempC = $data[4];
// HUMIDITY
(int)$Humidity = $data[5];
// WIND
$Wind = number_format($data[1] / 1.852, 1, '.', ''); // kts
$Gust = number_format($data[2] / 1.852, 1, '.', ''); // kts
(int)$WindDir = $data[3];
// BAROMETER
(float)$Baro = $data[6];
// RAIN
(float)$RainToday = $data[7];

//	SET @JOU = DAYOFYEAR(NOW());
$DST = date("Z")/3600;  // Decalage horaire
$JOU = date("z");       // Jour de l'année 0 365
$LAT = deg2rad( 42.7136788229883 );
$LON = deg2rad( 2.84930393099784 );
$SO  = 1367.6;
$HOD = (date("G")*60 + date("i") - .5) / 60; //tSV
$HRA = 2*pi()*($HOD - 12.0) / 24.0;          //Angle Horaire
$jd = GregorianToJD(date("m"), date("d"), date("Y"));
$HD = ((date("G")+((date('i') + date('s') / 60)/60))/24);
//correct for half-day offset
  $dayfrac = date('G') / 24 - .5;
  if ($dayfrac < 0) $dayfrac += 1;
  //now set the fraction of a day
  $frac = $dayfrac + (date('i') + date('s') / 60) / 60 / 24;
  $julianDate = $jd + $frac - $DST /24;
//echo "JD $julianDate<br>";
  $julianCentury = ($julianDate-2451545)/36525; //G2
//echo "JC $julianCentury<br>";
$GMLS = fmod(280.46646+$julianCentury*(36000.76983 + $julianCentury*0.0003032),360); //Geom Mean Long Sun (deg) I2
$GMAS = 357.52911+$julianCentury*(35999.05029 - 0.0001537*$julianCentury); //Geom Mean Anom Sun (deg)           J2
//echo "GMLS $GMLS/GMAS $GMAS<br>";
$EEO  = 0.016708634-$julianCentury*(0.000042037+0.0000001267*$julianCentury); //Eccent Earth Orbit              K2                                                                              K2
$SEC  = SIN(deg2rad($GMAS))*(1.914602-$julianCentury*(0.004817+0.000014*$julianCentury))+SIN(deg2rad(2*$GMAS))*(0.019993-0.000101*$julianCentury)+SIN(deg2rad(3*$GMAS))*0.000289;//Sun Eq of Ctr                                                                                                 L2
$STL  = $GMLS + $SEC; //Sun True Long (deg) M2
$STA  = $GMAS + $SEC; //Sun True Anom (deg) N2
$SRV  = (1.000001018*(1-$EEO*$EEO))/(1+$EEO*COS(deg2rad($STA))); //Sun Rad Vector (AUs) O2
//echo "SRV $SRV<br>";
$SAL  = $STL-0.00569-0.00478*SIN(deg2rad(125.04-1934.136*$julianCentury)); //Sun App Long (deg)   P2
$MOE  = 23+(26+((21.448-$julianCentury*(46.815+$julianCentury*(0.00059-$julianCentury*0.001813))))/60)/60; //Mean Obliq Ecliptic (deg) Q2
$OC   = $MOE+0.00256*COS(deg2rad(125.04-1934.136*$julianCentury)); //Obliq Corr (deg) R2
$SRA  = rad2deg(ATAN2(COS(deg2rad($OC))*SIN(deg2rad($SAL)),COS(deg2rad($SAL)))); //Sun Rt Ascen (deg) S2
$SD   = rad2deg(ASIN(SIN(deg2rad($OC))*SIN(deg2rad($SAL)))); //Sun Declin (deg) T2 -> DES
//echo "SD $SD<br>";
$VY   = TAN(deg2rad($OC/2))*TAN(deg2rad($OC/2)); // var y U2
//echo "SRA $SRA VY $VY<br>";
// Eq of Time (minutes) V2
$EQT  = 4*rad2deg($VY*SIN(2*deg2rad($GMLS))-2*$EEO*SIN(deg2rad($GMAS))+4*$EEO*$VY*SIN(deg2rad($GMAS))*COS(2*deg2rad($GMLS))-0.5*$VY*$VY*SIN(4*deg2rad($GMLS))-1.25*$EEO*$EEO*SIN(2*deg2rad($GMAS)));
//echo "EQT $EQT<br>";
// HA Sunrise (deg) W2
//      DEGRES(ACOS(COS(RADIANS(90,833))/(COS(RADIANS($B$3))*COS(RADIANS(T136)))-TAN(RADIANS($B$3))*TAN(RADIANS(T136))))
$HASR = rad2deg(ACOS(COS(deg2rad(90.833))/(COS(($LAT))*COS(deg2rad($SD)))-TAN(($LAT))*TAN(deg2rad($SD))));
//echo "HASR $HASR<br>";
$QN   = (720-4*rad2deg($LON)-$EQT+$DST*60)/1440;//Solar Noon (LST)  X2
$SRT  = $QN-$HASR*4/1440; //Sunrise Time (LST) Y2
$SST  = $QN+$HASR*4/1440; //Sunset Time (LST)  Z2
$SLD  = 8*$HASR; //Sunlight Duration (minutes) AA2
$TST  = fmod($HD*1440+$EQT+4*rad2deg($LON)-60*$DST,1440); //True Solar Time (min) AB2
$HA   = ($TST/4<0)? $TST/4+180: $TST/4-180; //Hour Angle (deg)  AC2
$SZA  = rad2deg(ACOS(SIN(($LAT))*SIN(deg2rad($SD))+COS(($LAT))*COS(deg2rad($SD))*COS(deg2rad($HA))));//Solar Zenith Angle (deg)
//echo "TST $TST / SZA $SZA<br>";
$SEA  = 90 - $SZA; //Solar Elevation Angle (deg) AE2
// Approx Atmospheric Refraction (deg) AF2
switch($SEA) {
case ($SEA>85 && $SEA<90)    : $AAR  = 0; break;
case ($SEA<-0.575)           : $AAR  = (-20.772/TAN(deg2rad($SEA)))/3600; break;
case ($SEA>-0.575 && $SEA<5 ): $AAR  = (1735+$SEA*(-518.2+$SEA*(103.4+$SEA*(-12.79+$SEA*0.711))))/3600; break;
case ($SEA>5 && $SEA<85)     : $AAR  = (58.1/TAN(deg2rad($SEA))-0.07/pow(TAN(deg2rad($SEA)),3)+0.000086/pow(TAN(deg2rad($SEA)),5))/3600; break;
}
//echo "SEA: $SEA / AAR: $AAR<br>";
$SEC  = $SEA + $AAR; // Solar Elevation corrected for atm refraction (deg)
// Solar Azimuth Angle (deg cw from N)
$SAA  = ($HA>0)?fmod(rad2deg(ACOS(((SIN(($LAT))*COS(deg2rad($SZA)))-SIN(deg2rad($SD)))/(COS(($LAT))*SIN(deg2rad($SZA)))))+180,360):fmod(540-rad2deg(ACOS(((SIN(($LAT))*COS(deg2rad($SZA)))-SIN(deg2rad($SD)))/(COS(($LAT))*SIN(deg2rad($SZA))))),360);
//echo "SEC: $SEC / SAA: $SAA<br>";

$FE = 1. + 0.0334*COS(2.*PI()*($JOU-2.7206)/365.25);
//$DES = 0.4093*SIN($JOU*((2.*PI())/365.)-1.405);
//$H = (sin($LAT)*sin($DES)+cos($LAT)*cos($DES)*cos($HRA));
//$HS = rad2deg($H);
$H  = deg2rad($SEC);
$SOL = $SO*$FE*$H;
//pow( $a , 0.6 ) use: exp( 0.6 * log($a) )
$Alt = 54;
//$PAtm = (1013.25 * (1 - (0.0065 * $Alt) / 288.15)^5.255) / 100; // en Pa
$Pvs = 2.165 * exp(8.02 * log(1.098 + $TempC / 100));    // en mmHg (millimètre de mercure)
$Pv = $Pvs * $Humidity / 100;
//$m = $PAtm / (101325 * sin($HS) + 15198.75 * (3.885 - $HS)^(-1.253)); //masse d'air optique relative (m) h est la hauteur du soleil en degrés
//$m = (0.89^$Alt) / sin($H);
// http://www.cder.dz/download/Art9-4_5.pdf
// $m = (1 - $Alt/10000) / (sin($H) + 0.50572 * exp(-1.6364 * log(6.07995 + $H))); // formule 3
// http://www.cythelia.fr/images/file/Gisement-solaire_Alain%20Ricaud_Jan-2011.pdf
$m = exp((-$Alt/1000)/8434.5) / (sin($H) + 0.50572 * exp(-1.6364 * log(6.07995 + $H))); // formule 3
//echo "<br>MA".$m." ".(1/cos(deg2rad($SZA)));
//$ER = 1 / (0.9 * $m + 9.4); //épaisseur optique de Rayleigh (ER)
// http://www.cder.dz/download/Art9-4_5.pdf
$ER = 1/(6.55567 + 1.7513 * $m - 0.1202 * pow($m,2) + 0.0065 * pow($m,3) - 0.00013 * pow($m,4));
//echo "RAYLEIGH ".$ER." ";
$B = 0.075; // pour un lieu rural  0,02 pour un lieu situé en montagne 0,20 pour un lieu industriel (atmosphère polluée)
$TL = 2.4 + 14.6 * $B + 0.4 * (1 + 2 * $B) * log($Pv); //facteur de trouble de Linke
$SOL = $SOL * exp(-$ER * $m * $TL); // rayonnement solaire direct sur un plan récepteur normal à ce rayonnement en W/m²
if ($SOL <= 0 || $SOL > 3000) return "0.0";
//echo "<br>DIRECT ".$SOL;
//Albédo 0.22 //0 pour un plan horizontal 90 pour vertical
$DI = 125*exp(0.4*log(sin($H)))*((1+cos(deg2rad(0)))/2)+211.86*exp(1.22*log(sin($H)))*((1-cos(deg2rad(0)))/2);
if(is_nan($DI)) $DI=0;
//echo "<br>DIFFUS ".$DI;
$CI = 1; // Coefficient d'orientation à calculer
$SOL = ($cloud * $SOL*$CI) + (2 * $DI * $cloud);
// http://herve.silve.pagesperso-orange.fr/solaire.htm

//$DSO = ACOS(-TAN(($LAT))*TAN($DES));
return $solar_coord = array($SEC, $SAA, number_format($SOL, 1, '.', ''));

}

function strip_units($data) {
  	if (preg_match('/([\d\,\.\+\-]+)/', $data, $t))
  		return $t[1];
}

function soil() {
$NOW = getdate();
/*
     [seconds] => 40
     [minutes] => 58
     [hours]   => 21
     [mday]    => 17
     [wday]    => 2
     [mon]     => 6
     [year]    => 2003
     [yday]    => 167
     [weekday] => Tuesday
     [month]   => June
     [0]       => 1055901520
*/
$DATEHEURE = $NOW['year']."-".$NOW['mon']."-".$NOW['mday']." ".$NOW['hours'].":".$NOW['minutes'].":".$NOW['seconds'];

include 'c:\wdisplay\webfiles\testtags.php';
// http://www.environmentalbiophysics.org/textbook/chapter-4/  A refaire
// http://soilphysics.okstate.edu/software/SoilTemperature/document.pdf

$clientrawfile = 'c:\wdisplay\webfiles\clientraw.txt';
$dataraw = file_get_contents($clientrawfile);

// clean up any blank lines
$dataraw = trim($dataraw);
$dataraw = preg_replace("/[\r\n]+[\s\t]*[\r\n]+/","\n",$dataraw);
$raw = explode(" ", $dataraw);

$tablesoil = 'meteo.soil';
// Une augmentation de la teneur en sel de 1% environ diminue l'évaporation de 1%

$timex = 24 * (date("z") -1) + date("G") + date("i")/60;  //Jour de l'année-1+H24+Min/60
$Tau = 365.25 * 24;               // 8766 Periode d'un an et calcul horaire
$omega = 2 * pi() /$Tau;
$z = 0.25; // metre
$Df = 3;    // thermal conductivity ?  0.5->2.5 pour un sol naturel
$Cv = 2500.0; // volumetric heat capacity Cv  Mineral 2000 Matiere organique 2500 Eau 4200

$Zd = sqrt((2*$Df)/($Cv*$omega));echo "Zd ".$Zd."<br>"; // Profondeur amortissement
//$Zd = 0.369710711;   // A définir

$Tmoy = 16.45; // Moyenne 2011
$A    = 10.44 ; // Moyenne 2011

$lastavg = strtolower('avtemp'.date("M",mktime(0, 0, 0, date("m")-1, date("d"),   date("Y"))));

$TSol = $Tmoy+ $A * exp(-$z/$Zd) * cos($omega * ($timex-4382.88)  - $z/$Zd );
$T2 = 29.52*cos(1.566 + 39.89*$TSol)/$TSol - 1.629*cos(6.195 + 29.52*cos(1.566 + 39.89*$TSol)/$TSol);
$table = 'meteo.synop';
$lastm = sql_query("SELECT `TimeStamp`, `Piscine` FROM meteo.soil ORDER BY TimeStamp DESC LIMIT 1;");
            list($last, $piscine) = sql_fetch_row($lastm);

$Ta = $apparentsolartempc;
$DSpread = $piscine - $dew0minuteago;
$C  = ($apparentsolartempc - $temp0minuteago)-($piscine - $TSol)+(1-1/$DSpread);
$Trend = 0.001838940262*$C;  // from Nutioniam Eureqa Formulize Trend = f(C) Mois aout pas de 1mn

$F60 = $piscine+$Trend;
$Piscine2 = 19 + sin(5.415926536 + 0.05972544369*$apparentsolartempc) + tan(cos(4.884961621 + 2*$TSol) - 43.92468675*cos(2.905360109 + 0.01851851852*$dew0minuteago)) - 0.1409778526*$T2;
/*
$tau5 = $apparentsolartempc * 99/100;
$tau1 = $apparentsolartempc * 63/100;
//$tau  = -$timex / log(1-$apparentsolartempc/$F60);

$RC = $tau;

?>
