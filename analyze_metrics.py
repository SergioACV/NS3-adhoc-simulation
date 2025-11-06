#!/usr/bin/env python3
"""
Análisis Estadístico de Métricas NS-3 con Repeticiones
Calcula promedios, desviaciones estándar y genera gráficos avanzados
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import sys
import os

# Configurar estilo
sns.set_style("whitegrid")
plt.rcParams['figure.figsize'] = (16, 12)
plt.rcParams['font.size'] = 10

def load_and_group_metrics(csv_file):
    """Cargar métricas y agrupar por configuración"""
    if not os.path.exists(csv_file):
        print(f" Error: No se encontró el archivo {csv_file}")
        print(f"   Ejecuta primero: ./run_scenarios_extended.sh")
        sys.exit(1)
    
    df = pd.read_csv(csv_file)
    print(f" Datos cargados: {len(df)} experimentos individuales")
    
    # Extraer configuración (drones y lluvia) del nombre del escenario
    df['Num_Drones_Config'] = df['Escenario'].str.extract(r'(\d+)D_')[0]
    df['Condicion_Lluvia'] = df['Escenario'].str.extract(r'_(ConLluvia|SinLluvia)_')[0]
    df['Repeticion'] = df['Escenario'].str.extract(r'_Rep(\d+)')[0]
    
    # Filtrar filas con datos válidos (eliminar experimentos de prueba)
    valid_mask = df['Num_Drones_Config'].notna() & df['Condicion_Lluvia'].notna() & df['Repeticion'].notna()
    
    if not valid_mask.all():
        invalid_scenarios = df[~valid_mask]['Escenario'].tolist()
        print(f"\n  Advertencia: Se encontraron {len(invalid_scenarios)} experimentos con formato inválido:")
        for scenario in invalid_scenarios:
            print(f"   - {scenario}")
        print(f"   Estos experimentos serán ignorados en el análisis.")
        df = df[valid_mask].copy()
        print(f"\n Datos válidos: {len(df)} experimentos")
    
    # Convertir a tipos apropiados
    df['Num_Drones_Config'] = df['Num_Drones_Config'].astype(int)
    df['Repeticion'] = df['Repeticion'].astype(int)
    
    # Crear columna de configuración
    df['Config'] = df['Num_Drones_Config'].astype(str) + 'D_' + df['Condicion_Lluvia']
    
    print(f"\n Configuraciones encontradas:")
    configs = df.groupby('Config').size()
    for config, count in configs.items():
        print(f"   • {config}: {count} repeticiones")
    
    return df

def calculate_statistics(df):
    """Calcular estadísticas (promedio, std, min, max) por configuración"""
    
    # Columnas numéricas a analizar
    numeric_cols = [
        'PDR_%', 'Tasa_Perdida_%', 'Lat_Prom_s', 'Lat_Min_s', 'Lat_Max_s', 
        'Lat_StdDev_s', 'Conectividad_%', 'PDR_Lluvia_%', 'PDR_Sin_Lluvia_%',
        'ClusterA_PDR_%', 'ClusterA_Lat_s', 
        'ClusterB_PDR_%', 'ClusterB_Lat_s',
        'ClusterC_PDR_%', 'ClusterC_Lat_s',
        'Eficiencia_Drones_%', 'Paq_Enviados', 'Paq_Entregados'
    ]
    
    # Agrupar por configuración
    stats = df.groupby('Config')[numeric_cols].agg(['mean', 'std', 'min', 'max'])
    stats.columns = ['_'.join(col).strip() for col in stats.columns.values]
    
    return stats

def plot_pdr_with_error_bars(df, stats):
    """Gráfico 1: PDR con barras de error (desviación estándar)"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    
    # Separar por lluvia
    con_lluvia = stats.filter(like='ConLluvia', axis=0)
    sin_lluvia = stats.filter(like='SinLluvia', axis=0)
    
    # Extraer número de drones
    con_lluvia_drones = [int(idx.split('D_')[0]) for idx in con_lluvia.index]
    sin_lluvia_drones = [int(idx.split('D_')[0]) for idx in sin_lluvia.index]
    
    # Subplot 1: PDR por número de drones
    width = 1.2
    x1 = np.array(con_lluvia_drones) - width/2
    x2 = np.array(sin_lluvia_drones) + width/2
    
    ax1.bar(x1, con_lluvia['PDR_%_mean'], width, 
            yerr=con_lluvia['PDR_%_std'], 
            label='Con Lluvia', color='#3498db', alpha=0.8, capsize=5)
    ax1.bar(x2, sin_lluvia['PDR_%_mean'], width,
            yerr=sin_lluvia['PDR_%_std'],
            label='Sin Lluvia', color='#2ecc71', alpha=0.8, capsize=5)
    
    ax1.set_xlabel('Número de Drones', fontsize=12, fontweight='bold')
    ax1.set_ylabel('PDR (%)', fontsize=12, fontweight='bold')
    ax1.set_title('PDR Promedio por Cantidad de Drones', 
                  fontsize=14, fontweight='bold')
    ax1.set_xticks([3, 6, 9])
    ax1.axhline(y=90, color='orange', linestyle='--', linewidth=2, label='Objetivo: 90%')
    ax1.legend(fontsize=11)
    ax1.grid(axis='y', alpha=0.3)
    
    # Subplot 2: Comparación PDR con/sin lluvia
    configs_sorted = sorted(stats.index, key=lambda x: int(x.split('D_')[0]))
    x_pos = range(len(configs_sorted))
    
    pdr_means = [stats.loc[cfg, 'PDR_%_mean'] for cfg in configs_sorted]
    pdr_stds = [stats.loc[cfg, 'PDR_%_std'] for cfg in configs_sorted]
    colors = ['#e74c3c' if 'ConLluvia' in cfg else '#27ae60' for cfg in configs_sorted]
    
    bars = ax2.bar(x_pos, pdr_means, yerr=pdr_stds, color=colors, alpha=0.7, capsize=5, edgecolor='black')
    ax2.set_xlabel('Configuración', fontsize=12, fontweight='bold')
    ax2.set_ylabel('PDR (%)', fontsize=12, fontweight='bold')
    ax2.set_title('PDR por Configuración', fontsize=14, fontweight='bold')
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels([cfg.replace('_', '\n') for cfg in configs_sorted], rotation=0, fontsize=9)
    ax2.grid(axis='y', alpha=0.3)
    
    # Añadir valores sobre las barras
    for i, (mean, std) in enumerate(zip(pdr_means, pdr_stds)):
        ax2.text(i, mean + std + 1, f'{mean:.1f}±{std:.1f}', 
                ha='center', va='bottom', fontsize=9, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig('grafico_pdr_estadistico.png', dpi=300, bbox_inches='tight')
    print(" Gráfico guardado: grafico_pdr_estadistico.png")

def plot_latency_analysis(df, stats):
    """Gráfico 2: Análisis de latencia con tendencias"""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
    
    # Subplot 1: Latencia vs número de drones
    for lluvia in ['ConLluvia', 'SinLluvia']:
        subset = stats.filter(like=lluvia, axis=0)
        drones = sorted([int(idx.split('D_')[0]) for idx in subset.index])
        lat_means = [subset.loc[f'{d}D_{lluvia}', 'Lat_Prom_s_mean'] for d in drones]
        lat_stds = [subset.loc[f'{d}D_{lluvia}', 'Lat_Prom_s_std'] for d in drones]
        
        color = '#3498db' if lluvia == 'ConLluvia' else '#2ecc71'
        label = 'Con Lluvia' if lluvia == 'ConLluvia' else 'Sin Lluvia'
        
        ax1.errorbar(drones, lat_means, yerr=lat_stds, marker='o', linewidth=2, 
                    markersize=8, capsize=5, label=label, color=color)
    
    ax1.set_xlabel('Número de Drones', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Latencia Promedio (s)', fontsize=12, fontweight='bold')
    ax1.set_title('Latencia vs Cantidad de Drones', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.set_xticks([3, 6, 9])
    
    # Subplot 2: Box plot de latencias
    configs_sorted = sorted(stats.index, key=lambda x: (int(x.split('D_')[0]), x))
    latency_data = []
    labels = []
    
    for config in configs_sorted:
        df_config = df[df['Config'] == config]
        latency_data.append(df_config['Lat_Prom_s'].values)
        labels.append(config.replace('_', '\n'))
    
    bp = ax2.boxplot(latency_data, labels=labels, patch_artist=True)
    for patch, config in zip(bp['boxes'], configs_sorted):
        color = '#e74c3c' if 'ConLluvia' in config else '#27ae60'
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax2.set_xlabel('Configuración', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Latencia (s)', fontsize=12, fontweight='bold')
    ax2.set_title('Distribución de Latencias', fontsize=14, fontweight='bold')
    ax2.grid(axis='y', alpha=0.3)
    plt.setp(ax2.xaxis.get_majorticklabels(), rotation=45, ha='right', fontsize=9)
    
    # Subplot 3: Latencia por cluster
    clusters = ['A', 'B', 'C']
    x_pos = np.arange(len(clusters))
    width = 0.15
    
    configs_to_plot = ['3D_SinLluvia', '6D_SinLluvia', '9D_SinLluvia']
    colors_clusters = ['#e74c3c', '#3498db', '#2ecc71']
    
    for i, config in enumerate(configs_to_plot):
        if config in stats.index:
            lat_means = [stats.loc[config, f'Cluster{c}_Lat_s_mean'] for c in clusters]
            lat_stds = [stats.loc[config, f'Cluster{c}_Lat_s_std'] for c in clusters]
            offset = (i - 1) * width
            ax3.bar(x_pos + offset, lat_means, width, yerr=lat_stds,
                   label=config, color=colors_clusters[i], alpha=0.8, capsize=3)
    
    ax3.set_xlabel('Cluster', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Latencia Promedio (s)', fontsize=12, fontweight='bold')
    ax3.set_title('Latencia por Cluster', fontsize=14, fontweight='bold')
    ax3.set_xticks(x_pos)
    ax3.set_xticklabels(clusters)
    ax3.legend(fontsize=10)
    ax3.grid(axis='y', alpha=0.3)
    
    # Subplot 4: Tabla de estadísticas de latencia
    ax4.axis('tight')
    ax4.axis('off')
    
    table_data = []
    for config in configs_sorted:
        row = [
            config.replace('_', ' '),
            f"{stats.loc[config, 'Lat_Prom_s_mean']:.3f}",
            f"{stats.loc[config, 'Lat_Prom_s_std']:.3f}",
            f"{stats.loc[config, 'Lat_Min_s_mean']:.3f}",
            f"{stats.loc[config, 'Lat_Max_s_mean']:.3f}"
        ]
        table_data.append(row)
    
    table = ax4.table(cellText=table_data,
                     colLabels=['Config', 'Lat Media (s)', 'Std Dev (s)', 'Min (s)', 'Max (s)'],
                     cellLoc='center',
                     loc='center',
                     colWidths=[0.25, 0.2, 0.2, 0.15, 0.15])
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 2)
    
    # Colorear encabezado
    for i in range(5):
        table[(0, i)].set_facecolor('#34495e')
        table[(0, i)].set_text_props(weight='bold', color='white')
    
    ax4.set_title('Estadísticas Detalladas de Latencia', fontsize=14, fontweight='bold', pad=20)
    
    plt.tight_layout()
    plt.savefig('grafico_latencia_estadistico.png', dpi=300, bbox_inches='tight')
    print(" Gráfico guardado: grafico_latencia_estadistico.png")

def plot_cluster_comparison(df, stats):
    """Gráfico 3: Comparación entre clusters"""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
    
    clusters = ['A', 'B', 'C']
    configs_test = ['3D_ConLluvia', '6D_ConLluvia', '9D_ConLluvia']
    
    # Subplot 1: PDR por cluster
    x_pos = np.arange(len(clusters))
    width = 0.25
    colors = ['#e74c3c', '#3498db', '#2ecc71']
    
    for i, config in enumerate(configs_test):
        if config in stats.index:
            pdr_means = [stats.loc[config, f'Cluster{c}_PDR_%_mean'] for c in clusters]
            pdr_stds = [stats.loc[config, f'Cluster{c}_PDR_%_std'] for c in clusters]
            offset = (i - 1) * width
            ax1.bar(x_pos + offset, pdr_means, width, yerr=pdr_stds,
                   label=config.replace('_', ' '), color=colors[i], alpha=0.8, capsize=3)
    
    ax1.set_xlabel('Cluster', fontsize=12, fontweight='bold')
    ax1.set_ylabel('PDR (%)', fontsize=12, fontweight='bold')
    ax1.set_title('PDR por Cluster', fontsize=14, fontweight='bold')
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels(clusters)
    ax1.legend(fontsize=10)
    ax1.grid(axis='y', alpha=0.3)
    ax1.axhline(y=90, color='orange', linestyle='--', label='Objetivo 90%')
    
    # Subplot 2: Latencia inter-cluster simulada
    # (Nota: Aquí podríamos calcular diferencias de latencia entre clusters)
    configs_no_rain = ['3D_SinLluvia', '6D_SinLluvia', '9D_SinLluvia']
    
    for config in configs_no_rain:
        if config in stats.index:
            lats = [stats.loc[config, f'Cluster{c}_Lat_s_mean'] for c in clusters]
            drones_num = int(config.split('D_')[0])
            ax2.plot(clusters, lats, marker='o', linewidth=2, markersize=8, label=f'{drones_num} Drones')
    
    ax2.set_xlabel('Cluster', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Latencia Promedio (s)', fontsize=12, fontweight='bold')
    ax2.set_title('Latencia por Cluster', fontsize=14, fontweight='bold')
    ax2.legend(fontsize=11)
    ax2.grid(True, alpha=0.3)
    
    # Subplot 3: Heatmap de PDR por cluster y drones
    heatmap_data = []
    for config in sorted(stats.index, key=lambda x: int(x.split('D_')[0])):
        if 'ConLluvia' in config:
            row = [stats.loc[config, f'Cluster{c}_PDR_%_mean'] for c in clusters]
            heatmap_data.append(row)
    
    sns.heatmap(heatmap_data, annot=True, fmt='.1f', cmap='RdYlGn', vmin=0, vmax=100,
                xticklabels=clusters, yticklabels=['3D', '6D', '9D'],
                cbar_kws={'label': 'PDR (%)'}, ax=ax3)
    ax3.set_title('PDR por Cluster y Configuración', fontsize=14, fontweight='bold')
    ax3.set_xlabel('Cluster', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Configuración', fontsize=12, fontweight='bold')
    
    # Subplot 4: Variabilidad (Desviación Estándar de PDR)
    std_data = []
    labels_std = []
    colors_std = []
    
    for config in sorted(stats.index, key=lambda x: (int(x.split('D_')[0]), 'ConLluvia' in x)):
        for cluster in clusters:
            std_val = stats.loc[config, f'Cluster{cluster}_PDR_%_std']
            std_data.append(std_val)
            labels_std.append(f'{config.split("D_")[0]}D\n{cluster}')
            # Asignar color basado en la condición de lluvia
            color = '#e74c3c' if 'ConLluvia' in config else '#27ae60'
            colors_std.append(color)
    
    # Verificar si hay datos de variabilidad (std > 0)
    max_std = max(std_data) if std_data else 0
    
    if max_std > 0:
        ax4.bar(range(len(std_data)), std_data, color=colors_std, alpha=0.7, edgecolor='black')
        ax4.set_ylabel('Desviación Estándar PDR (%)', fontsize=12, fontweight='bold')
        ax4.set_title('Variabilidad de PDR por Cluster', fontsize=14, fontweight='bold')
        ax4.axhline(y=5, color='orange', linestyle='--', label='Límite aceptable: 5%')
    else:
        # Si no hay variabilidad, mostrar mensaje
        ax4.text(0.5, 0.5, 'Se requieren múltiples repeticiones\npara calcular la variabilidad', 
                ha='center', va='center', transform=ax4.transAxes, fontsize=12, 
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        ax4.set_title('Variabilidad de PDR por Cluster (Sin datos)', fontsize=14, fontweight='bold')
    
    ax4.set_xlabel('Configuración - Cluster', fontsize=12, fontweight='bold')
    ax4.set_xticks(range(len(labels_std)))
    ax4.set_xticklabels(labels_std, rotation=45, ha='right', fontsize=7)
    ax4.grid(axis='y', alpha=0.3)
    ax4.legend()
    
    plt.tight_layout()
    plt.savefig('grafico_clusters_estadistico.png', dpi=300, bbox_inches='tight')
    print(" Gráfico guardado: grafico_clusters_estadistico.png")

def plot_rain_impact_corrected(df, stats):
    """Gráfico 4: Impacto de lluvia"""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
    
    # Subplot 1: Comparación PDR GLOBAL de escenarios con/sin lluvia
    drones_configs = [3, 6, 9]
    x_pos = np.arange(len(drones_configs))
    width = 0.35
    
    pdr_escenarios_lluvia = []  # PDR global de escenarios CON lluvia
    pdr_escenarios_sin = []     # PDR global de escenarios SIN lluvia
    std_lluvia = []
    std_sin = []
    
    for d in drones_configs:
        config_lluvia = f'{d}D_ConLluvia'
        config_sin = f'{d}D_SinLluvia'
        
        # Usar PDR GLOBAL (PDR_%) no PDR específico de lluvia
        if config_lluvia in stats.index:
            pdr_escenarios_lluvia.append(stats.loc[config_lluvia, 'PDR_%_mean'])
            std_lluvia.append(stats.loc[config_lluvia, 'PDR_%_std'])
        else:
            pdr_escenarios_lluvia.append(0)
            std_lluvia.append(0)
            
        if config_sin in stats.index:
            pdr_escenarios_sin.append(stats.loc[config_sin, 'PDR_%_mean'])
            std_sin.append(stats.loc[config_sin, 'PDR_%_std'])
        else:
            pdr_escenarios_sin.append(0)
            std_sin.append(0)
    
    ax1.bar(x_pos - width/2, pdr_escenarios_lluvia, width, 
            yerr=std_lluvia, label='Escenario con lluvia', 
            color='#3498db', alpha=0.8, capsize=5)
    ax1.bar(x_pos + width/2, pdr_escenarios_sin, width, 
            yerr=std_sin, label='Escenario sin lluvia',
            color='#2ecc71', alpha=0.8, capsize=5)
    
    ax1.set_xlabel('Número de Drones', fontsize=12, fontweight='bold')
    ax1.set_ylabel('PDR (%)', fontsize=12, fontweight='bold')
    ax1.set_title('Comparación PDR: Escenarios Con/Sin Lluvia', fontsize=14, fontweight='bold')
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels(drones_configs)
    ax1.legend(fontsize=11)
    ax1.grid(axis='y', alpha=0.3)
    
    # Calcular degradación (diferencia entre escenarios)
    for i, d in enumerate(drones_configs):
        if pdr_escenarios_lluvia[i] > 0 and pdr_escenarios_sin[i] > 0:
            degradacion = pdr_escenarios_sin[i] - pdr_escenarios_lluvia[i]
            if degradacion > 0:
                ax1.text(i, max(pdr_escenarios_lluvia[i], pdr_escenarios_sin[i]) + 2,
                        f'-{degradacion:.1f}%', ha='center', color='red', fontweight='bold')
            else:
                ax1.text(i, max(pdr_escenarios_lluvia[i], pdr_escenarios_sin[i]) + 2,
                        f'+{abs(degradacion):.1f}%', ha='center', color='green', fontweight='bold')
    
    # Subplot 2: Tasa de pérdida de paquetes
    configs_sorted = sorted(stats.index, key=lambda x: (int(x.split('D_')[0]), x))
    loss_means = [stats.loc[cfg, 'Tasa_Perdida_%_mean'] for cfg in configs_sorted]
    loss_stds = [stats.loc[cfg, 'Tasa_Perdida_%_std'] for cfg in configs_sorted]
    colors = ['#e74c3c' if 'ConLluvia' in cfg else '#27ae60' for cfg in configs_sorted]
    
    x_pos_loss = range(len(configs_sorted))
    ax2.bar(x_pos_loss, loss_means, yerr=loss_stds, color=colors, alpha=0.7, capsize=5, edgecolor='black')
    ax2.set_xlabel('Configuración', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Tasa de Pérdida (%)', fontsize=12, fontweight='bold')
    ax2.set_title('Tasa de Pérdida de Paquetes', fontsize=14, fontweight='bold')
    ax2.set_xticks(x_pos_loss)
    ax2.set_xticklabels([cfg.replace('_', '\n') for cfg in configs_sorted], rotation=0, fontsize=9)
    ax2.grid(axis='y', alpha=0.3)
    
    # Subplot 3: Eficiencia de drones
    efficiency_means = [stats.loc[cfg, 'Eficiencia_Drones_%_mean'] for cfg in configs_sorted]
    efficiency_stds = [stats.loc[cfg, 'Eficiencia_Drones_%_std'] for cfg in configs_sorted]
    
    ax3.bar(x_pos_loss, efficiency_means, yerr=efficiency_stds, color=colors, alpha=0.7, capsize=5, edgecolor='black')
    ax3.set_xlabel('Configuración', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Eficiencia (%)', fontsize=12, fontweight='bold')
    ax3.set_title('Eficiencia de Respuesta de Drones', fontsize=14, fontweight='bold')
    ax3.set_xticks(x_pos_loss)
    ax3.set_xticklabels([cfg.replace('_', '\n') for cfg in configs_sorted], rotation=0, fontsize=9)
    ax3.grid(axis='y', alpha=0.3)
    ax3.axhline(y=80, color='orange', linestyle='--', label='Objetivo: 80%')
    ax3.legend()
    
    # Subplot 4: Conectividad
    conn_means = [stats.loc[cfg, 'Conectividad_%_mean'] for cfg in configs_sorted]
    conn_stds = [stats.loc[cfg, 'Conectividad_%_std'] for cfg in configs_sorted]
    
    ax4.bar(x_pos_loss, conn_means, yerr=conn_stds, color=colors, alpha=0.7, capsize=5, edgecolor='black')
    ax4.set_xlabel('Configuración', fontsize=12, fontweight='bold')
    ax4.set_ylabel('Conectividad (%)', fontsize=12, fontweight='bold')
    ax4.set_title('Tasa de Conectividad CH-Drone', fontsize=14, fontweight='bold')
    ax4.set_xticks(x_pos_loss)
    ax4.set_xticklabels([cfg.replace('_', '\n') for cfg in configs_sorted], rotation=0, fontsize=9)
    ax4.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('grafico_impacto_lluvia.png', dpi=300, bbox_inches='tight')
    print(" Gráfico guardado: grafico_impacto_lluvia.png")

def generate_summary_report(stats):
    """Generar reporte HTML con resumen estadístico"""
    html = """
    <html>
    <head>
        <title>Reporte Estadístico - NS3 Ad Hoc Network</title>
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; background-color: #f5f5f5; }
            h1 { color: #2c3e50; text-align: center; }
            h2 { color: #34495e; border-bottom: 2px solid #3498db; padding-bottom: 10px; }
            table { border-collapse: collapse; width: 100%; background-color: white; margin: 20px 0; }
            th { background-color: #3498db; color: white; padding: 12px; text-align: left; }
            td { padding: 10px; border-bottom: 1px solid #ddd; }
            tr:hover { background-color: #f5f5f5; }
            .metric-excellent { color: #27ae60; font-weight: bold; }
            .metric-good { color: #2980b9; font-weight: bold; }
            .metric-warning { color: #f39c12; font-weight: bold; }
            .metric-bad { color: #e74c3c; font-weight: bold; }
            .summary-box { background-color: white; padding: 15px; margin: 10px 0; border-left: 4px solid #3498db; }
        </style>
    </head>
    <body>
        <h1> Reporte Estadístico Completo</h1>
        <h2>🔍 Resumen Ejecutivo</h2>
        <div class="summary-box">
            <p><strong>Total de configuraciones probadas:</strong> """ + str(len(stats)) + """</p>
            <p><strong>Repeticiones por configuración:</strong> 5</p>
            <p><strong>Cantidades de drones probadas:</strong> 3, 6, 9</p>
        </div>
        
        <h2> Estadísticas Detalladas</h2>
    """
    
    html += stats.to_html(float_format='%.3f')
    
    html += """
        <h2> Conclusiones Clave</h2>
        <div class="summary-box">
            <ul>
                <li>Las métricas ahora separan correctamente PDR con/sin lluvia</li>
                <li>La latencia se analiza con min, max y desviación estándar</li>
                <li>Cada cluster (A, B, C) tiene métricas independientes</li>
                <li>Los experimentos se repitieron 5 veces para confiabilidad estadística</li>
            </ul>
        </div>
    </body>
    </html>
    """
    
    with open('reporte_estadistico.html', 'w') as f:
        f.write(html)
    
    print(" Reporte HTML guardado: reporte_estadistico.html")

def main():
    print("\n" + "="*70)
    print("  ANÁLISIS ESTADÍSTICO - Simulación NS-3 con Repeticiones")
    print("="*70 + "\n")
    
    # Cargar y procesar datos
    csv_file = 'metricas_simulacion.csv'
    df = load_and_group_metrics(csv_file)
    
    # Calcular estadísticas
    print("\n Calculando estadísticas (promedio, std, min, max)...")
    stats = calculate_statistics(df)
    
    # Guardar estadísticas en CSV
    stats.to_csv('estadisticas_resumidas.csv')
    print(f" Estadísticas guardadas en: estadisticas_resumidas.csv")
    
    # Generar gráficos
    print("\n Generando gráficos estadísticos...\n")
    plot_pdr_with_error_bars(df, stats)
    plot_latency_analysis(df, stats)
    plot_cluster_comparison(df, stats)
    plot_rain_impact_corrected(df, stats)
    generate_summary_report(stats)
    
    print("\n" + "="*70)
    print(" ANÁLISIS COMPLETADO")
    print("="*70)
    print("\n📁 Archivos generados:")
    print("   • grafico_pdr_estadistico.png")
    print("   • grafico_latencia_estadistico.png")
    print("   • grafico_clusters_estadistico.png")
    print("   • grafico_impacto_lluvia.png")
    print("   • estadisticas_resumidas.csv")
    print("   • reporte_estadistico.html")
    print("\n Los gráficos incluyen barras de error y estadísticas robustas\n")

if __name__ == "__main__":
    main()
