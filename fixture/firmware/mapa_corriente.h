// ============================================================
//  MAPA ESTÁTICO PWM -> CORRIENTE (rotor bloqueado)
//  Generado a partir de datos/estatica_20261027_093128.csv
//  (01_identificacion_corriente). Para regenerarlo con datos
//  nuevos usa matlab/generar_mapa.m
//
//  IN1 (+): medido de 0 a 600; de 650 a 1023 extrapolado con
//           i = 38.47 + 1.2603e-3 (u - 49.86)^2
//  IN2 (-): PROVISIONAL = IN1 con 0.88 de ganancia (razón de
//           los K de los escalones). Mide ESTATICA 0 -600 -50 y
//           regenera el mapa.
//
//  El controlador usa la inversa: u = mapa^-1(w), con w en mA.
// ============================================================
#pragma once

const int MAP_N = 22;

const float MAP_U[MAP_N] = {
  0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500,
  550, 600, 650, 700, 750, 800, 850, 900, 950, 1000, 1023 };

const float MAP_I_POS[MAP_N] = {
  30.22, 30.88, 41.28, 51.11, 66.46, 88.86, 117.73, 152.36, 193.58, 240.64, 293.35,
  351.05, 421.87, 492.39, 571.18, 656.26, 747.65, 845.34, 949.34, 1059.63, 1176.23, 1231.98 };

const float MAP_I_NEG[MAP_N] = {
  30.22, 30.80, 39.95, 48.60, 62.11, 81.82, 107.23, 137.70, 173.98, 215.39, 261.77,
  312.55, 374.87, 436.93, 506.26, 581.14, 661.56, 747.53, 839.04, 936.10, 1038.71, 1087.77 };
