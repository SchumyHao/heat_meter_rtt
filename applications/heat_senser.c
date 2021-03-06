/*
 * File      : heat_senser.c
 *
 * This file impliment the thread process senser data about heat meter.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-4-22      Schumy       First version
 */

#include <rtthread.h>
#include <stdio.h>
//#include "finsh.h"

#include "spi_tdc_gp21.h"
#include "board.h"

#define HS_DEBUG_PRINT              rt_kprintf
//#define HS_DEBUG_PRINT(...)
#define HS_TOF_PRINT_EPD            1
#define HS_TEMP_PRINT_EPD           1

#define DIAMETER_MM                (20.0F)
#define THETA                      (30)
#define SIN_2THETA                 (0.866F)
#define DISTANCE_MM                (4.0F)

struct hm_print_data {
    rt_uint16_t x;
    rt_uint16_t y;
    const char* str;
};

struct hm_tof_data {
    struct spi_tdc_gp21_tof_data data;
    float speed;      //m^3/h
    rt_tick_t time;
};

struct hm_temp_data {
    struct spi_tdc_gp21_temp_data data;
    float hot;       //��
    float cold;      //��
    rt_tick_t time;
};

#if (HM_BOARD_UART_6 == 1)
#define TOF_TEMP_PRINT_DEVICE       "uart6"
static rt_device_t tt_print_device;
#endif

#define TOF_DATA_BUF_LEN            (30)
static struct hm_tof_data tof_data[TOF_DATA_BUF_LEN];
#define TEMP_DATA_BUF_LEN           (5)
static struct hm_temp_data temp_data[TEMP_DATA_BUF_LEN];

#define TDC_SLEEP_TIME_MS           (500)

static rt_event_t cal_event;
#define TDC_DATA_FULL_EVENT         (1U<<0)

static rt_mutex_t tof_lock;
static rt_mutex_t temp_lock;
#define LOCK_TACK_WAIT_TIME_MS      (1500)

#define PT1000_MIN_TEMP             (0)
#define PT1000_MAX_TEMP             (100)
const static float PT1000[][10] = {
    /*        0.0         0.1         0.2         0.3         0.4         0.5         0.6         0.7         0.8         0.9  */
    /*0*/  {1000.000,   1000.391,   1000.782,   1001.172,   1001.563,   1001.954,   1002.345,   1002.736,   1003.126,   1003.517},
    /*1*/  {1003.908,   1004.298,   1004.689,   1005.080,   1005.470,   1005.861,   1006.252,   1006.642,   1007.033,   1007.424},
    /*2*/  {1007.814,   1008.205,   1008.595,   1008.986,   1009.377,   1009.767,   1010.158,   1010.548,   1010.939,   1011.320},
    /*3*/  {1011.720,   1012.110,   1012.501,   1012.891,   1013.282,   1013.672,   1014.060,   1014.453,   1014.843,   1015.230},
    /*4*/  {1015.624,   1016.014,   1016.405,   1016.795,   1017.185,   1017.576,   1017.966,   1018.356,   1018.747,   1019.130},
    /*5*/  {1019.527,   1019.917,   1020.308,   1020.698,   1021.088,   1021.478,   1021.868,   1022.259,   1022.649,   1023.030},
    /*6*/  {1023.429,   1023.819,   1024.209,   1024.599,   1024.989,   1025.380,   1025.770,   1026.160,   1026.550,   1026.940},
    /*7*/  {1027.330,   1027.720,   1028.110,   1028.500,   1028.890,   1029.280,   1029.670,   1030.060,   1030.450,   1030.840},
    /*8*/  {1031.229,   1031.619,   1032.009,   1032.399,   1032.789,   1033.179,   1033.569,   1033.958,   1034.348,   1034.738},
    /*9*/  {1035.128,   1035.518,   1035.907,   1036.297,   1036.687,   1037.077,   1037.466,   1037.856,   1038.246,   1038.636},
    /*10*/ {1039.025,   1039.415,   1039.805,   1040.194,   1040.584,   1040.973,   1041.363,   1041.753,   1042.142,   1042.530},
    /*11*/ {1042.921,   1043.311,   1043.701,   1044.090,   1044.480,   1044.869,   1045.250,   1045.648,   1046.038,   1046.427},
    /*12*/ {1046.816,   1047.206,   1047.595,   1047.985,   1048.374,   1048.764,   1049.153,   1049.542,   1049.932,   1050.321},
    /*13*/ {1050.710,   1051.099,   1051.489,   1051.878,   1052.268,   1052.657,   1053.040,   1053.435,   1053.825,   1054.210},
    /*14*/ {1054.603,   1054.992,   1055.381,   1055.771,   1056.160,   1056.549,   1056.930,   1057.327,   1057.716,   1058.105},
    /*15*/ {1058.495,   1058.884,   1059.273,   1059.662,   1060.051,   1060.440,   1060.829,   1061.218,   1061.607,   1061.996},
    /*16*/ {1062.385,   1062.774,   1063.163,   1063.552,   1063.941,   1064.330,   1064.719,   1065.108,   1065.496,   1065.885},
    /*17*/ {1066.274,   1066.663,   1067.052,   1067.441,   1067.830,   1068.218,   1068.607,   1068.996,   1069.385,   1069.774},
    /*18*/ {1070.162,   1070.551,   1070.940,   1071.328,   1071.717,   1072.106,   1072.495,   1072.883,   1073.272,   1073.661},
    /*19*/ {1074.049,   1074.438,   1074.826,   1075.215,   1075.604,   1075.992,   1076.381,   1076.769,   1077.158,   1077.546},
    /*20*/ {1077.935,   1078.324,   1078.712,   1079.101,   1079.489,   1079.877,   1080.266,   1080.654,   1081.043,   1081.430},
    /*21*/ {1081.820,   1082.208,   1082.596,   1082.985,   1083.373,   1083.762,   1084.150,   1084.538,   1084.926,   1085.310},
    /*22*/ {1085.703,   1086.091,   1086.480,   1086.868,   1087.256,   1087.644,   1088.033,   1088.421,   1088.809,   1089.197},
    /*23*/ {1089.585,   1089.974,   1090.362,   1090.750,   1091.138,   1091.526,   1091.914,   1092.302,   1092.690,   1093.078},
    /*24*/ {1093.467,   1093.855,   1094.243,   1094.631,   1095.019,   1095.407,   1095.795,   1096.183,   1096.571,   1096.959},
    /*25*/ {1097.347,   1097.734,   1098.122,   1098.510,   1098.898,   1099.286,   1099.670,   1100.062,   1100.450,   1100.838},
    /*26*/ {1101.225,   1101.613,   1102.001,   1102.389,   1102.777,   1103.164,   1103.550,   1103.940,   1104.328,   1104.710},
    /*27*/ {1105.103,   1105.491,   1105.879,   1106.266,   1106.654,   1107.042,   1107.429,   1107.817,   1108.204,   1108.590},
    /*28*/ {1108.980,   1109.367,   1109.755,   1110.142,   1110.530,   1110.917,   1111.305,   1111.693,   1112.080,   1112.468},
    /*29*/ {1112.855,   1113.242,   1113.630,   1114.017,   1114.405,   1114.792,   1115.180,   1115.567,   1115.954,   1116.342},
    /*30*/ {1116.729,   1117.117,   1117.504,   1117.891,   1118.279,   1118.666,   1119.053,   1119.441,   1119.828,   1120.215},
    /*31*/ {1120.602,   1120.990,   1121.377,   1121.764,   1122.151,   1122.538,   1122.920,   1123.313,   1123.700,   1124.087},
    /*32*/ {1124.474,   1124.861,   1125.248,   1125.636,   1126.023,   1126.410,   1126.797,   1127.184,   1127.571,   1127.958},
    /*33*/ {1128.345,   1128.732,   1129.119,   1130.127,   1129.893,   1130.280,   1130.667,   1131.054,   1131.441,   1131.820},
    /*34*/ {1132.215,   1132.602,   1132.988,   1133.375,   1133.762,   1134.149,   1134.536,   1134.923,   1135.309,   1135.690},
    /*35*/ {1136.083,   1136.470,   1136.857,   1137.243,   1137.630,   1138.017,   1138.404,   1138.790,   1139.177,   1139.564},
    /*36*/ {1139.950,   1140.337,   1140.724,   1141.110,   1141.497,   1141.884,   1142.270,   1142.657,   1143.043,   1143.430},
    /*37*/ {1143.817,   1144.203,   1144.590,   1144.976,   1145.363,   1145.749,   1146.136,   1146.522,   1146.909,   1147.295},
    /*38*/ {1147.681,   1148.068,   1148.454,   1148.841,   1149.227,   1149.614,   1150.000,   1150.386,   1150.773,   1151.159},
    /*39*/ {1151.545,   1151.932,   1152.318,   1152.704,   1153.091,   1153.477,   1153.860,   1154.249,   1154.636,   1155.020},
    /*40*/ {1155.408,   1155.794,   1156.180,   1156.567,   1156.953,   1157.339,   1157.725,   1158.111,   1158.497,   1158.883},
    /*41*/ {1159.270,   1159.656,   1160.042,   1160.428,   1160.814,   1161.200,   1161.586,   1161.972,   1162.358,   1162.744},
    /*42*/ {1163.130,   1163.516,   1163.902,   1164.288,   1164.674,   1165.060,   1165.446,   1165.831,   1166.217,   1166.600},
    /*43*/ {1166.989,   1167.375,   1167.761,   1168.147,   1168.532,   1168.918,   1169.304,   1169.690,   1170.076,   1170.460},
    /*44*/ {1170.847,   1171.233,   1171.619,   1172.004,   1172.390,   1172.776,   1173.161,   1173.547,   1173.933,   1174.318},
    /*45*/ {1174.704,   1175.090,   1175.475,   1175.861,   1176.247,   1176.632,   1177.018,   1177.403,   1177.789,   1178.174},
    /*46*/ {1178.560,   1178.945,   1179.331,   1179.716,   1180.102,   1180.487,   1180.873,   1181.258,   1181.644,   1182.029},
    /*47*/ {1182.414,   1182.800,   1183.185,   1183.571,   1183.956,   1184.341,   1184.727,   1185.112,   1185.597,   1185.883},
    /*48*/ {1186.268,   1186.653,   1187.038,   1187.424,   1187.809,   1188.194,   1188.579,   1188.965,   1189.350,   1189.735},
    /*49*/ {1190.120,   1190.505,   1190.890,   1191.276,   1191.661,   1192.046,   1192.431,   1192.816,   1193.201,   1193.586},
    /*50*/ {1193.971,   1194.356,   1194.741,   1195.126,   1195.511,   1195.896,   1196.281,   1196.666,   1197.051,   1197.436},
    /*51*/ {1197.821,   1198.206,   1198.591,   1198.976,   1199.361,   1199.746,   1200.131,   1200.516,   1200.900,   1201.285},
    /*52*/ {1201.670,   1202.055,   1202.440,   1202.824,   1203.209,   1203.594,   1203.979,   1204.364,   1204.748,   1205.133},
    /*53*/ {1205.518,   1205.902,   1206.287,   1206.672,   1207.056,   1207.441,   1207.826,   1208.210,   1208.595,   1208.980},
    /*54*/ {1209.364,   1209.749,   1210.133,   1210.518,   1210.902,   1211.287,   1211.670,   1212.056,   1212.441,   1212.825},
    /*55*/ {1213.210,   1213.594,   1213.978,   1214.363,   1214.747,   1215.120,   1215.516,   1215.901,   1216.285,   1216.669},
    /*56*/ {1217.054,   1217.438,   1217.822,   1218.207,   1218.591,   1218.975,   1219.360,   1219.744,   1220.128,   1220.513},
    /*57*/ {1220.897,   1221.281,   1221.665,   1222.049,   1222.434,   1222.818,   1223.202,   1223.586,   1223.970,   1224.350},
    /*58*/ {1224.739,   1225.123,   1225.507,   1225.891,   1226.275,   1226.659,   1227.043,   1227.427,   1227.811,   1228.195},
    /*59*/ {1228.579,   1228.963,   1229.347,   1229.731,   1230.115,   1230.499,   1230.883,   1231.267,   1231.651,   1232.030},
    /*60*/ {1232.419,   1232.803,   1233.187,   1233.571,   1233.955,   1234.338,   1234.722,   1235.106,   1235.490,   1235.870},
    /*61*/ {1236.257,   1236.641,   1237.025,   1237.409,   1237.792,   1238.176,   1238.560,   1238.944,   1239.327,   1239.711},
    /*62*/ {1240.095,   1240.478,   1240.862,   1241.246,   1241.629,   1242.030,   1242.396,   1242.780,   1243.164,   1243.500},
    /*63*/ {1243.931,   1244.314,   1244.698,   1245.081,   1245.465,   1245.848,   1246.230,   1246.615,   1246.999,   1247.382},
    /*64*/ {1247.766,   1248.149,   1248.533,   1248.916,   1249.299,   1249.683,   1250.066,   1250.450,   1250.833,   1251.210},
    /*65*/ {1251.600,   1251.983,   1252.366,   1252.749,   1253.133,   1253.516,   1253.899,   1254.283,   1254.666,   1255.040},
    /*66*/ {1255.432,   1255.815,   1256.199,   1256.582,   1256.965,   1257.348,   1257.731,   1258.114,   1258.497,   1258.880},
    /*67*/ {1259.264,   1259.647,   1260.030,   1260.413,   1260.796,   1261.179,   1261.562,   1261.945,   1262.328,   1262.710},
    /*68*/ {1263.094,   1263.477,   1263.860,   1264.243,   1264.626,   1265.009,   1265.392,   1265.775,   1266.157,   1266.540},
    /*69*/ {1266.923,   1267.306,   1267.689,   1268.072,   1268.455,   1268.837,   1269.220,   1269.603,   1269.986,   1270.360},
    /*70*/ {1270.751,   1271.134,   1271.517,   1271.899,   1272.282,   1272.665,   1273.048,   1273.430,   1273.813,   1274.190},
    /*71*/ {1274.578,   1274.691,   1274.803,   1274.916,   1275.029,   1275.141,   1275.254,   1275.366,   1275.479,   1275.591},
    /*72*/ {1278.404,   1278.786,   1279.169,   1279.551,   1279.934,   1280.316,   1280.699,   1281.081,   1281.464,   1281.846},
    /*73*/ {1282.228,   1282.611,   1282.993,   1283.376,   1283.758,   1284.140,   1284.523,   1284.905,   1285.287,   1285.670},
    /*74*/ {1286.052,   1286.434,   1286.816,   1287.199,   1287.581,   1287.963,   1288.345,   1288.728,   1289.110,   1289.492},
    /*75*/ {1289.874,   1290.256,   1290.638,   1291.021,   1291.403,   1291.785,   1292.167,   1292.549,   1292.931,   1293.313},
    /*76*/ {1293.695,   1294.077,   1294.459,   1294.841,   1295.223,   1295.605,   1295.980,   1296.369,   1296.751,   1297.130},
    /*77*/ {1297.515,   1297.897,   1298.279,   1298.661,   1299.043,   1299.425,   1299.807,   1300.188,   1300.570,   1300.952},
    /*78*/ {1301.334,   1301.716,   1302.098,   1302.479,   1302.861,   1303.243,   1303.625,   1304.006,   1304.388,   1304.770},
    /*79*/ {1305.152,   1305.533,   1305.915,   1306.297,   1306.678,   1307.060,   1307.442,   1307.823,   1308.205,   1308.586},
    /*80*/ {1308.968,   1309.350,   1309.731,   1310.113,   1310.494,   1310.876,   1311.270,   1311.639,   1312.020,   1312.402},
    /*81*/ {1312.783,   1313.165,   1313.546,   1313.928,   1314.309,   1314.691,   1315.072,   1315.453,   1315.835,   1316.216},
    /*82*/ {1316.597,   1316.979,   1317.360,   1317.742,   1318.123,   1318.504,   1318.885,   1319.267,   1319.648,   1320.029},
    /*83*/ {1320.411,   1320.792,   1321.173,   1321.554,   1321.935,   1322.316,   1322.697,   1323.079,   1323.460,   1323.841},
    /*84*/ {1324.222,   1324.603,   1324.985,   1325.366,   1325.747,   1326.128,   1326.509,   1326.890,   1327.271,   1327.652},
    /*85*/ {1328.033,   1328.414,   1328.795,   1329.176,   1329.557,   1329.938,   1330.319,   1330.700,   1331.081,   1331.460},
    /*86*/ {1331.843,   1332.224,   1332.604,   1332.985,   1333.366,   1333.747,   1334.128,   1334.509,   1334.889,   1335.270},
    /*87*/ {1335.651,   1336.032,   1336.413,   1336.793,   1337.174,   1337.555,   1337.935,   1338.316,   1338.697,   1339.070},
    /*88*/ {1339.458,   1335.839,   1332.220,   1328.600,   1324.981,   1321.361,   1317.742,   1314.123,   1310.503,   1306.884},
    /*89*/ {1343.264,   1343.645,   1344.025,   1344.406,   1344.786,   1345.167,   1345.570,   1345.928,   1346.308,   1346.689},
    /*90*/ {1347.069,   1347.450,   1347.830,   1348.211,   1348.591,   1348.971,   1349.352,   1349.732,   1350.112,   1350.493},
    /*91*/ {1350.873,   1351.253,   1351.634,   1352.014,   1352.394,   1352.774,   1353.155,   1353.535,   1353.915,   1354.295},
    /*92*/ {1354.676,   1355.056,   1355.436,   1355.816,   1356.196,   1356.577,   1356.957,   1357.337,   1357.717,   1358.097},
    /*93*/ {1358.477,   1358.857,   1359.237,   1359.617,   1359.997,   1360.377,   1360.757,   1361.137,   1361.517,   1361.897},
    /*94*/ {1362.277,   1362.657,   1363.037,   1363.417,   1363.797,   1364.177,   1364.557,   1364.937,   1365.317,   1365.690},
    /*95*/ {1366.077,   1366.456,   1366.836,   1367.216,   1367.596,   1367.976,   1368.355,   1368.735,   1369.115,   1369.495},
    /*96*/ {1369.875,   1370.254,   1370.634,   1371.014,   1371.393,   1371.773,   1372.153,   1372.532,   1372.912,   1373.290},
    /*97*/ {1373.671,   1374.051,   1374.431,   1374.810,   1375.190,   1375.569,   1375.949,   1376.329,   1376.708,   1377.088},
    /*98*/ {1377.467,   1377.847,   1378.226,   1378.606,   1378.985,   1379.365,   1379.744,   1380.123,   1380.503,   1380.882},
    /*99*/ {1381.262,   1381.641,   1382.020,   1382.400,   1382.779,   1383.158,   1383.538,   1383.917,   1384.296,   1384.676}
};

const static float density[] = {
    /*       1        2        3        4        5        6       7        8        9        10 */
    /*0*/   1000.70,1000.70,1000.70,1000.70,1000.70,1000.70,1000.60,1000.60,1000.50,1000.40,
    /*10*/  1000.30,1000.20,1000.10,999.95,999.80,999.64,999.47,999.29,999.10,998.89,
    /*20*/  998.68,998.45,998.22,997.98,997.72,997.46,997.19,996.91,996.62,996.32,
    /*30*/  996.01,995.69,995.37,995.04,994.69,994.35,993.99,993.62,993.25,992.87,
    /*40*/  992.49,992.09,991.69,991.28,990.87,990.44,990.02,989.58,989.14,988.69,
    /*50*/  988.23,987.77,987.30,986.83,986.35,985.86,985.37,984.87,984.36,983.85,
    /*60*/  983.33,982.81,982.28,981.75,981.21,980.66,980.11,979.55,978.99,978.43,
    /*70*/  977.85,977.27,976.69,976.10,975.51,974.91,974.30,973.70,973.08,972.46,
    /*80*/  971.84,971.76,970.21,969.93,969.29,968.64,967.99,967.33,966.66,965.99,
    /*90*/  965.32,964.64,963.96,963.27,962.58,961.88,961.18,960.48,959.77,959.05,
    /*100*/ 958.33,957.61,956.88,956.15,955.41,954.67,953.92,953.17,952.41,951.65,
    /*110*/ 950.89,950.12,949.34,948.57,947.78,947.00,946.21,945.41,944.61,943.81,
    /*120*/ 943.00,942.19,941.37,940.55,939.72,938.89,938.06,937.22,936.37,935.52,
    /*130*/ 934.67,933.82,932.95,932.09,931.22,930.35,929.47,928.58,927.70,926.81,
    /*140*/ 925.91,925.01,924.10,923.19,922.28,921.36,920.44,919.51,918.58,917.65
};

const static float enthalpy[] = {
    /*      1       2      3       4       5       6      7       8      9       10 */
    /*0*/   5.7964,10.004,14.209,18.411,22.611,26.808,31.004,35.197,39.389,43.579,
    /*10*/  47.768,51.956,56.142,60.327,64.511,68.693,72.875,77.057,81.237,85.417,
    /*20*/  89.596,93.774,97.952,102.13,106.31,110.48,114.66,118.84,123.01,127.19,
    /*30*/  131.36,135.54,139.72,143.89,148.07,152.24,156.42,160.59,164.77,168.94,
    /*40*/  173.12,177.30,181.47,185.65,189.82,194.00,198.18,202.36,206.53,210.71,
    /*50*/  214.89,219.07,223.25,227.42,231.60,235.78,239.96,244.14,248.33,252.51,
    /*60*/  256.69,260.87,265.05,269.24,273.42,277.61,281.79,285.98,290.16,294.35,
    /*70*/  298.54,302.72,306.91,311.10,315.29,319.48,323.67,327.86,332.06,336.25,
    /*80*/  340.44,344.64,348.83,353.03,357.23,361.42,365.62,369.82,374.02,378.22,
    /*90*/  382.43,386.63,390.83,395.04,399.24,403.45,407.66,411.87,416.08,420.29,
    /*100*/ 424.51,428.72,432.93,437.15,441.37,445.59,449.81,454.03,458.25,462.48,
    /*110*/ 466.70,470.93,475.16,479.39,483.62,487.85,492.08,496.32,500.56,504.80,
    /*120*/ 509.04,513.28,517.52,521.77,526.02,530.27,534.52,538.77,543.03,547.28,
    /*130*/ 551.54,555.80,560.07,564.33,568.60,572.87,577.14,581.41,585.69,589.96,
    /*140*/ 594.24,598.53,602.81,607.10,611.39,615.68,619.97,624.27,628.57,632.87
};

#if (HM_BOARD_UART_6 == 1)
static void hm_ble_write(const char* str, rt_size_t size)
{
    rt_size_t writen = size;
    const char* ptr = str;

    while(writen) {
        if(writen < 20) {
            rt_device_write(tt_print_device, 0, ptr, writen);
            writen = 0;
        }
        else {
            rt_device_write(tt_print_device, 0, ptr, 20);
            writen -= 20;
            ptr += 20;
            rt_thread_delay(2);
        }
    }
}
#endif

extern rt_err_t hm_print(struct hm_print_data* pd);
rt_inline void heat_print(float heat)
{
    static struct hm_print_data pd;
    static char buf[10];
    sprintf(buf, "%.2f", heat);
    pd.x = 36;
    pd.y = 7;
    pd.str = buf;
    hm_print(&pd);
}

rt_inline void tof_print(struct hm_tof_data* data)
{
    static char ble_buf[100];
    sprintf(ble_buf, "%ld:tof up=%f down=%f speed=%f\n\r",
            data->time, data->data.up, data->data.down, data->speed);
#if (HM_BOARD_UART_6 == 1)
    hm_ble_write(ble_buf, rt_strlen(ble_buf)+1);
#else
    rt_kprintf(ble_buf, rt_strlen(ble_buf)+1);
#endif
}

#if HS_TOF_PRINT_EPD
rt_inline void tof_print_epd(float data)
{
    static struct hm_print_data pd;
    static char buf[10];
    sprintf(buf, "%2.3f", data);
    pd.x = 36;
    pd.y = 12;
    pd.str = buf;
    hm_print(&pd);
}
#endif

rt_inline void temp_print(struct hm_temp_data* data)
{
    static char ble_buf[100];
    sprintf(ble_buf, "%ld:temp R_hot=%f R_cold=%f hot=%f cold=%f\n\r",
            data->time, data->data.hot, data->data.cold, data->hot, data->cold);
#if (HM_BOARD_UART_6 == 1)
    hm_ble_write(ble_buf, rt_strlen(ble_buf)+1);
#else
    rt_kprintf(ble_buf, rt_strlen(ble_buf)+1);
#endif
}

#if HS_TEMP_PRINT_EPD
rt_inline void temp_print_epd(struct hm_temp_data* data)
{
#define   HOT    (0)
#define   COLD   (1)
    static struct hm_print_data pd[2];
    static char buf[2][10];
    sprintf(buf[HOT], "%2.1f", data->hot);
    pd[HOT].x = 36;
    pd[HOT].y = 2;
    pd[HOT].str = buf[HOT];
    hm_print(&pd[HOT]);
    sprintf(buf[COLD], "%2.1f", data->cold);
    pd[COLD].x = 115;
    pd[COLD].y = 2;
    pd[COLD].str = buf[COLD];
    hm_print(&pd[COLD]);
}
#endif

void hm_tdc_thread_entry(void* parameter)
{
    rt_device_t tdc = RT_NULL;
    rt_uint8_t tof_data_count = 0;
    rt_uint8_t temp_data_count = 0;
    rt_uint8_t loop_count = TOF_DATA_BUF_LEN/TEMP_DATA_BUF_LEN;


    tdc = rt_device_find(HM_BOARD_TDC_NAME);
    RT_ASSERT(tdc);
    rt_device_open(tdc, RT_DEVICE_OFLAG_RDWR);

#if (HM_BOARD_UART_6 == 1)
    tt_print_device = rt_device_find(TOF_TEMP_PRINT_DEVICE);
    RT_ASSERT(tt_print_device);
    rt_device_open(tt_print_device, RT_DEVICE_OFLAG_RDWR);
#endif

    tof_lock = rt_mutex_create("L_tof", RT_IPC_FLAG_FIFO);
    RT_ASSERT(tof_lock);
    temp_lock = rt_mutex_create("L_temp", RT_IPC_FLAG_FIFO);
    RT_ASSERT(temp_lock);

    while(1) {
        loop_count--;
        { /* TOF */
            if(rt_mutex_take(tof_lock, rt_tick_from_millisecond(LOCK_TACK_WAIT_TIME_MS)) != RT_EOK) {
                rt_kprintf("TOF take lock error\n");
                continue;
            }
            else {
                rt_device_control(tdc, SPI_TDC_GP21_CTRL_MEASURE_TOF2, &tof_data[tof_data_count].data);
                tof_data[tof_data_count].time = rt_tick_get();
                rt_mutex_release(tof_lock);
                tof_data_count++;
            }
        }

        if(loop_count == 0) { /* TEMP */
            loop_count = TOF_DATA_BUF_LEN/TEMP_DATA_BUF_LEN;
            if(rt_mutex_take(temp_lock, rt_tick_from_millisecond(LOCK_TACK_WAIT_TIME_MS)) != RT_EOK) {
                rt_kprintf("temprature take lock error\n");
                continue;
            }
            else {
                rt_device_control(tdc, SPI_TDC_GP21_CTRL_MEASURE_TEMP, &temp_data[temp_data_count].data);
                temp_data[temp_data_count].time = rt_tick_get();
                rt_mutex_release(temp_lock);
                temp_data_count++;
            }
        }

        if((temp_data_count==TEMP_DATA_BUF_LEN) || (tof_data_count==TOF_DATA_BUF_LEN)) { /* Send event */
            temp_data_count = 0;
            tof_data_count = 0;
            if(rt_event_send(cal_event, TDC_DATA_FULL_EVENT) != RT_EOK) {
                rt_kprintf("TDC send event error\n");
            }
        }

        rt_thread_delay(rt_tick_from_millisecond(TDC_SLEEP_TIME_MS));
    }
}

static int bi_search_pt_coarse(const int res)
{
    int mid = 0;
    int begin = 0;
    int end = sizeof(PT1000)/sizeof(PT1000[0]);

    const float (*ptr)[sizeof(PT1000[0])/sizeof(PT1000[0][0])];
    ptr = PT1000;

    while(begin <= end) {
        mid = (begin+end)/2;
        if(res > (int)(ptr[mid][0])) {
            begin = mid + 1;
        }
        else if(res < (int)(ptr[mid][0])) {
            end = mid - 1;
        }
        else {
            return mid;
        }
    }
    return end;
}

static int bi_search_pt_fine(const float res, const int coarse)
{
    int mid = 0;
    int begin = 0;
    int end = sizeof(PT1000[0])/sizeof(PT1000[0][0]);

    const float* ptr = PT1000[coarse];

    while(begin <= end) {
        mid = (begin+end)/2;
        if(res > ptr[mid]) {
            begin = mid + 1;
        }
        else if(res < ptr[mid]) {
            end = mid - 1;
        }
        else {
            return mid;
        }
    }
    return end;
}

static float temp_cal(const float temp_res)
{
    int coarse = bi_search_pt_coarse((int)temp_res);
    int fine = bi_search_pt_fine(temp_res, coarse);

    //TODO: why need -1?
    //return (1.0*(coarse+PT1000_MIN_TEMP)+0.1*fine);
    return ((1.0*(coarse+PT1000_MIN_TEMP)+0.1*fine)-1.0);
}

#include <finsh.h>
static long tmp_tof_cal_k = 18000;
FINSH_VAR_EXPORT(tmp_tof_cal_k, finsh_type_long, TOF);
static float tof_cal(const struct spi_tdc_gp21_tof_data* data)
{
    double d_t = data->up - data->down;  //us
    double a_t = (data->up + data->down)/2;  //us
    /* speed cal
       D/(2*sin*cos) * d_t/a_t^2
       D/sin2 * d_t/a_t^2

       but in real heat meter, use U type sensor, L = DISTANCE_MM
       d_t = 2*DISTANCE_MM*V/(V0^2)
       a_t = (DISTANCE_MM)/(V0)
       V = d_t*DISTANCE_MM /(2*a_t^2)  (m/ms)
       V = k * d_t/(a_t^2)  (m/s)
       k = DISTANCE_MM*1000/2
    */
#define TOF_CAL_K     (DISTANCE_MM*500.0)
    return ((float)tmp_tof_cal_k*d_t/(a_t*a_t));  //m^3/h
}

static float heat_cal(const float qv, const int time, const float hot, const float cold)
{
    float p_hot = 0;
    float h_hot = 0;
    float p_cold = 0;
    float h_cold = 0;

    RT_ASSERT((hot>0)&&(cold>0));
    p_hot = density[(int)hot-1];
    h_hot = enthalpy[(int)hot-1];
    p_cold = density[(int)cold-1];
    h_cold = enthalpy[(int)cold-1];

    /*
          qv   m^3/h
          time ms
          p    kg/m^3
          h    kj/kg
      */
    return (((p_hot*h_hot-p_cold*h_cold)/3600.0)*(qv*((float)time/1000.0)));
}

#if 0
rt_inline float qv_cal(const float speed)
{
    //m^3/s
    /* speed = m/s
       qv = speed/k * pi*d^2/4
       k = 4/3 pi = 3.14 d = DIAMETER_MM
    */
    return (0.023562*speed);
}
#endif

rt_inline int ticks_to_ms(const rt_tick_t tick)
{
#define TICK_TO_MS   (1000/RT_TICK_PER_SECOND)
    return tick*TICK_TO_MS;
}

static void find_temp(const rt_tick_t tick, float* hot, float* cold)
{
    int i = 0;

    if(tick <= temp_data[0].time) {
        *hot = temp_data[0].hot;
        *cold = temp_data[0].cold;
        return;
    }

    for(i=1; i<TEMP_DATA_BUF_LEN; i++) {
        if(tick <= temp_data[i].time) {
            *hot = temp_data[i].hot;
            *cold = temp_data[i].cold;
            return;
        }
    }

    *hot = temp_data[TEMP_DATA_BUF_LEN-1].hot;
    *cold = temp_data[TEMP_DATA_BUF_LEN-1].cold;
    return;
}


#ifndef ABS
#define ABS(a)     (((a)>0)? (a): (-(a)))
#endif
static float do_heat_cal(void)
{
    float hot = 0;
    float cold = 0;
    float heat = 0;
    float speed_avr = 0;
    static float speed_avr_pre = 0;
    float max_tolerance = 0;
    int i = 0;
    int count = 0;
    
    for(i=0; i<TEMP_DATA_BUF_LEN; i++) { /* cal temp */
        temp_data[i].hot = temp_cal(temp_data[i].data.hot);
        temp_data[i].cold = temp_cal(temp_data[i].data.cold);
#if 1
        temp_print(&temp_data[i]);
#endif
    }

    {/* cal speed */
    for(i=0; i<TOF_DATA_BUF_LEN; i++) {
        tof_data[i].speed = tof_cal(&tof_data[i].data);
        if((ABS(tof_data[i].speed) < 1.5) && 
           (tof_data[i].speed > 0)){
            speed_avr += tof_data[i].speed;
            count++;
        }
    }
    speed_avr = (count!=0)? (speed_avr/count): speed_avr_pre;
    }
    
    max_tolerance = 0.3 * speed_avr;
    max_tolerance = ABS(max_tolerance);
    for(i=0; i<TOF_DATA_BUF_LEN; i++) { /* remove wrong speed */
        if(ABS(speed_avr - tof_data[i].speed) > max_tolerance) {
            tof_data[i].speed = 0;
        }
    }

    {/* cal average speed again */
    count = 0;
    speed_avr = 0;
    for(i=0; i<TOF_DATA_BUF_LEN; i++) { 
        if(tof_data[i].speed != 0) {
            speed_avr += tof_data[i].speed;
            count++;
        }    
    }
    speed_avr = (count!=0)? (speed_avr/count): speed_avr_pre;
    speed_avr_pre = speed_avr;
    }

    for(i=0; i<TOF_DATA_BUF_LEN; i++) { /* replace wrong speed */
        if(tof_data[i].speed == 0) {
            tof_data[i].speed = speed_avr;
        }
    }
    
    for(i=0; i<TOF_DATA_BUF_LEN; i++) { /* correct small value */
        if((speed_avr < 0.15)&&(speed_avr > 0.065)) {
            tof_data[i].speed -= 0.06;
        }
        else if(speed_avr < 0.065) {
            tof_data[i].speed = 0;
        }
#if 1
        tof_print(&tof_data[i]);
#endif
    }
    if((speed_avr < 0.15)&&(speed_avr > 0.1)) {
        speed_avr -= 0.05;
    }
    if((speed_avr < 0.1)&&(speed_avr > 0.06)) {
        speed_avr -= 0.03;
    }
    else if(speed_avr < 0.06) {
        speed_avr = 0;
    }

#if 1
    temp_print_epd(&temp_data[0]);
    tof_print_epd(speed_avr);
#endif

    for(i=0; i<TOF_DATA_BUF_LEN-1; i++) {
        find_temp(tof_data[i].time, &hot, &cold);
        heat += heat_cal((tof_data[i].speed+tof_data[i+1].speed)/2,
                         ticks_to_ms(tof_data[i+1].time-tof_data[i].time),
                         hot, cold);
    }

    return heat; //kJ
}

void hm_heatcal_thread_entry(void* parameter)
{
    rt_uint32_t event_set = 0;
    float heat_used = 0;
    cal_event = rt_event_create("H_cal", RT_IPC_FLAG_FIFO);
    RT_ASSERT(cal_event);

    while(1) {
        if(rt_event_recv(cal_event, TDC_DATA_FULL_EVENT,
                         RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER,
                         &event_set)==RT_EOK) {
            if((rt_mutex_take(temp_lock, rt_tick_from_millisecond(LOCK_TACK_WAIT_TIME_MS)) ||
                (rt_mutex_take(tof_lock, rt_tick_from_millisecond(LOCK_TACK_WAIT_TIME_MS)))) != RT_EOK) {
                rt_kprintf("TOF and temprature take lock error\n");
            }
            heat_used += do_heat_cal();
            rt_mutex_release(tof_lock);
            rt_mutex_release(temp_lock);
            heat_print(heat_used);
        }
    }
}
