#!/bin/bash

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║  SIMULACIÓN NS-3: Análisis Estadístico con Repeticiones       ║"
echo "║  Pruebas: 3, 6, 9 drones | 5 repeticiones c/u | Con/Sin lluvia║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

if [ -f "metricas_simulacion.csv" ]; then
    echo "Eliminando archivo de métricas anterior..."
    rm metricas_simulacion.csv
fi

cd /home/csandovalc/projects/NS3-adhoc-simulation/ns-3.43
DRONES=(3 6 9)
REPETICIONES=5
TIEMPO_SIM=300

TOTAL_EXPERIMENTOS=$((${#DRONES[@]} * 2 * $REPETICIONES))
CONTADOR=0

echo ""
echo "CONFIGURACIÓN:"
echo "   Tiempo de simulación: ${TIEMPO_SIM}s por experimento"
echo "   Repeticiones por configuración: ${REPETICIONES}"
echo "   Configuraciones de drones: ${DRONES[@]}"
echo "   Total de experimentos: ${TOTAL_EXPERIMENTOS}"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

mostrar_progreso() {
    CONTADOR=$((CONTADOR + 1))
    PORCENTAJE=$((CONTADOR * 100 / TOTAL_EXPERIMENTOS))
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  Progreso: ${CONTADOR}/${TOTAL_EXPERIMENTOS} (${PORCENTAJE}%)"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
}

for NUM_DRONES in "${DRONES[@]}"; do
    
    echo "┌────────────────────────────────────────────────────────────────┐"
    echo "│  PROBANDO CON ${NUM_DRONES} DRONES                                        │"
    echo "└────────────────────────────────────────────────────────────────┘"
    echo ""
    
    echo "Escenarios CON LLUVIA (${NUM_DRONES} drones):"
    for REP in $(seq 1 $REPETICIONES); do
        mostrar_progreso
        ESCENARIO="${NUM_DRONES}D_ConLluvia_Rep${REP}"
        echo "   Ejecutando: ${ESCENARIO}..."
        ./ns3 run "scratch/adhoc --simTime=${TIEMPO_SIM} --nRecolector=${NUM_DRONES} --enableRain=true --scenario=${ESCENARIO}" > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ${ESCENARIO} completado"
        else
            echo "   ${ESCENARIO} falló"
        fi
    done
    
    echo ""
    
    echo "Escenarios SIN LLUVIA (${NUM_DRONES} drones):"
    for REP in $(seq 1 $REPETICIONES); do
        mostrar_progreso
        ESCENARIO="${NUM_DRONES}D_SinLluvia_Rep${REP}"
        echo "   Ejecutando: ${ESCENARIO}..."
        ./ns3 run "scratch/adhoc --simTime=${TIEMPO_SIM} --nRecolector=${NUM_DRONES} --enableRain=false --scenario=${ESCENARIO}" > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ${ESCENARIO} completado"
        else
            echo "   ${ESCENARIO} falló"
        fi
    done
    
    echo ""
done

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "TODOS LOS EXPERIMENTOS COMPLETADOS"
echo ""

if [ -f "metricas_simulacion.csv" ]; then
    echo "Archivo de métricas generado: metricas_simulacion.csv"
    TOTAL_LINEAS=$(wc -l < metricas_simulacion.csv)
    echo "   Total de registros: $((TOTAL_LINEAS - 1)) (sin contar encabezado)"
    echo ""
    echo "═══════════════════════════════════════════════════════════════"
    echo "                PRIMERAS 10 LÍNEAS DEL CSV"
    echo "═══════════════════════════════════════════════════════════════"
    echo ""
    head -n 11 metricas_simulacion.csv | column -t -s,
    echo ""
    echo "═══════════════════════════════════════════════════════════════"
    echo ""
else
    echo "No se generó el archivo de métricas"
    exit 1
fi

echo ""
echo "SIGUIENTE PASO: Analizar los datos"
echo ""
echo "   Opción 1 - Generar gráficos con Python:"
echo "   $ python3 analyze_metrics_extended.py"
echo ""
echo "   Opción 2 - Ver resumen estadístico:"
echo "   $ python3 statistical_summary.py"
echo ""
echo "   Opción 3 - Abrir CSV en LibreOffice/Excel:"
echo "   $ libreoffice metricas_simulacion.csv"
echo ""
echo "Los datos ya incluyen:"
echo "   Métricas detalladas de latencia (min, max, std dev)"
echo "   Tasa de pérdida de paquetes"
echo "   PDR corregido con/sin lluvia"
echo "   Métricas separadas por cluster (A, B, C)"
echo "   Eficiencia de drones"
echo ""
