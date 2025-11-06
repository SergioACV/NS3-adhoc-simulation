#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import sys
import os

sns.set_style("whitegrid")
plt.rcParams['figure.figsize'] = (18, 14)
plt.rcParams['font.size'] = 10

def load_metrics(csv_file):
    if not os.path.exists(csv_file):
        print(f"Error: No se encontró el archivo {csv_file}")
        sys.exit(1)
    
    df = pd.read_csv(csv_file)
    
    df = df[~df['Escenario'].str.contains('Test', case=False, na=False)]
    
    df['Num_Drones'] = df['Escenario'].str.extract(r'(\d+)D_')[0].fillna(0).astype(int)
    df['Condicion'] = df['Escenario'].str.extract(r'_(ConLluvia|SinLluvia)_')[0].fillna('Desconocido')
    df['Rep'] = df['Escenario'].str.extract(r'_Rep(\d+)')[0].fillna(1).astype(int)
    
    df = df[df['Num_Drones'] > 0]
    
    print(f"Datos cargados: {len(df)} experimentos válidos")
    print(f"\nResumen:")
    print(f"   Drones probados: {sorted(df['Num_Drones'].unique())}")
    print(f"   Condiciones: {df['Condicion'].unique().tolist()}")
    print(f"   Repeticiones: {df['Rep'].nunique()}")
    
    return df

def plot_comprehensive_analysis(df):
    fig = plt.figure(figsize=(20, 12))
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
    
    stats = df.groupby(['Num_Drones', 'Condicion']).agg({
        'PDR_%': ['mean', 'std'],
        'Lat_Prom_s': ['mean', 'std'],
        'Conectividad_%': ['mean', 'std'],
        'Paq_Enviados': 'mean',
        'Paq_Entregados': 'mean',
        'Tasa_Perdida_%': ['mean', 'std'],
        'Eficiencia_Drones_%': ['mean', 'std']
    }).reset_index()
    
    ax1 = fig.add_subplot(gs[0, 0])
    for cond in df['Condicion'].unique():
        data = stats[stats['Condicion'] == cond]
        ax1.errorbar(data['Num_Drones'], data[('PDR_%', 'mean')], 
                    yerr=data[('PDR_%', 'std')], marker='o', linewidth=2,
                    markersize=8, capsize=5, label=cond)
    ax1.set_xlabel('Número de Drones', fontsize=12, fontweight='bold')
    ax1.set_ylabel('PDR (%)', fontsize=12, fontweight='bold')
    ax1.set_title('PDR vs Número de Drones', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_xticks(sorted(df['Num_Drones'].unique()))
    
    ax2 = fig.add_subplot(gs[0, 1])
    width = 0.35
    drones_vals = sorted(df['Num_Drones'].unique())
    x = np.arange(len(drones_vals))
    
    enviados = [stats[stats['Num_Drones'] == d][('Paq_Enviados', 'mean')].mean() for d in drones_vals]
    entregados = [stats[stats['Num_Drones'] == d][('Paq_Entregados', 'mean')].mean() for d in drones_vals]
    
    ax2.bar(x - width/2, enviados, width, label='Enviados', color='#3498db', alpha=0.8)
    ax2.bar(x + width/2, entregados, width, label='Entregados', color='#2ecc71', alpha=0.8)
    ax2.set_xlabel('Número de Drones', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Cantidad de Paquetes', fontsize=12, fontweight='bold')
    ax2.set_title('Paquetes Enviados vs Entregados', fontsize=14, fontweight='bold')
    ax2.set_xticks(x)
    ax2.set_xticklabels(drones_vals)
    ax2.legend()
    ax2.grid(axis='y', alpha=0.3)
    
    for i, (env, ent) in enumerate(zip(enviados, entregados)):
        ax2.text(i - width/2, env + 2, f'{env:.0f}', ha='center', fontsize=9)
        ax2.text(i + width/2, ent + 2, f'{ent:.0f}', ha='center', fontsize=9)
    
    ax3 = fig.add_subplot(gs[0, 2])
    for cond in df['Condicion'].unique():
        data = stats[stats['Condicion'] == cond]
        ax3.errorbar(data['Num_Drones'], data[('Lat_Prom_s', 'mean')],
                    yerr=data[('Lat_Prom_s', 'std')], marker='s', linewidth=2,
                    markersize=8, capsize=5, label=cond)
    ax3.set_xlabel('Número de Drones', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Latencia Promedio (s)', fontsize=12, fontweight='bold')
    ax3.set_title('Latencia vs Número de Drones', fontsize=14, fontweight='bold')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.set_xticks(sorted(df['Num_Drones'].unique()))
    
    ax4 = fig.add_subplot(gs[1, 0])
    df_melted = df.copy()
    df_melted['Config'] = df_melted['Num_Drones'].astype(str) + 'D\n' + df_melted['Condicion']
    
    configs = sorted(df_melted['Config'].unique())
    data_for_box = [df_melted[df_melted['Config'] == c]['PDR_%'].values for c in configs]
    
    bp = ax4.boxplot(data_for_box, labels=configs, patch_artist=True)
    for patch, config in zip(bp['boxes'], configs):
        color = '#e74c3c' if 'ConLluvia' in config else '#27ae60'
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax4.set_ylabel('PDR (%)', fontsize=12, fontweight='bold')
    ax4.set_title('Distribución de PDR', fontsize=14, fontweight='bold')
    ax4.grid(axis='y', alpha=0.3)
    plt.setp(ax4.xaxis.get_majorticklabels(), rotation=45, ha='right', fontsize=9)
    
    ax5 = fig.add_subplot(gs[1, 1])
    for cond in df['Condicion'].unique():
        data = stats[stats['Condicion'] == cond]
        ax5.errorbar(data['Num_Drones'], data[('Conectividad_%', 'mean')],
                    yerr=data[('Conectividad_%', 'std')], marker='D', linewidth=2,
                    markersize=8, capsize=5, label=cond)
    ax5.set_xlabel('Número de Drones', fontsize=12, fontweight='bold')
    ax5.set_ylabel('Conectividad (%)', fontsize=12, fontweight='bold')
    ax5.set_title('Conectividad CH-Drone', fontsize=14, fontweight='bold')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    ax5.set_xticks(sorted(df['Num_Drones'].unique()))
    
    ax6 = fig.add_subplot(gs[1, 2])
    for cond in df['Condicion'].unique():
        data = stats[stats['Condicion'] == cond]
        ax6.errorbar(data['Num_Drones'], data[('Tasa_Perdida_%', 'mean')],
                    yerr=data[('Tasa_Perdida_%', 'std')], marker='v', linewidth=2,
                    markersize=8, capsize=5, label=cond)
    ax6.set_xlabel('Número de Drones', fontsize=12, fontweight='bold')
    ax6.set_ylabel('Tasa de Pérdida (%)', fontsize=12, fontweight='bold')
    ax6.set_title('Tasa de Pérdida de Paquetes', fontsize=14, fontweight='bold')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    ax6.set_xticks(sorted(df['Num_Drones'].unique()))
    
    ax7 = fig.add_subplot(gs[2, :])
    ax7.axis('tight')
    ax7.axis('off')
    
    table_data = []
    for _, row in stats.iterrows():
        drones = int(row['Num_Drones'])
        cond = row['Condicion']
        table_data.append([
            f"{drones}D {cond}",
            f"{row[('PDR_%', 'mean')]:.2f}±{row[('PDR_%', 'std')]:.2f}",
            f"{row[('Lat_Prom_s', 'mean')]:.2f}±{row[('Lat_Prom_s', 'std')]:.2f}",
            f"{row[('Conectividad_%', 'mean')]:.2f}±{row[('Conectividad_%', 'std')]:.2f}",
            f"{row[('Paq_Enviados', 'mean')]:.0f}",
            f"{row[('Paq_Entregados', 'mean')]:.0f}",
            f"{row[('Eficiencia_Drones_%', 'mean')]:.2f}±{row[('Eficiencia_Drones_%', 'std')]:.2f}"
        ])
    
    table = ax7.table(cellText=table_data,
                     colLabels=['Config', 'PDR (%)', 'Latencia (s)', 'Conectividad (%)', 
                               'Paq. Env.', 'Paq. Ent.', 'Efic. Drones (%)'],
                     cellLoc='center',
                     loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 2)
    
    for i in range(7):
        table[(0, i)].set_facecolor('#34495e')
        table[(0, i)].set_text_props(weight='bold', color='white')
    
    for i in range(1, len(table_data) + 1):
        if i % 2 == 0:
            for j in range(7):
                table[(i, j)].set_facecolor('#ecf0f1')
    
    ax7.set_title('Tabla Resumen de Estadísticas', fontsize=14, fontweight='bold', pad=20)
    
    plt.savefig('analisis_completo_mejorado.png', dpi=300, bbox_inches='tight')
    print("Gráfico guardado: analisis_completo_mejorado.png")

def generate_html_report(df):
    stats = df.groupby(['Num_Drones', 'Condicion']).agg({
        'PDR_%': ['mean', 'std', 'min', 'max'],
        'Lat_Prom_s': ['mean', 'std', 'min', 'max'],
        'Paq_Enviados': ['mean', 'sum'],
        'Paq_Entregados': ['mean', 'sum'],
        'Conectividad_%': ['mean', 'std'],
        'Tasa_Perdida_%': ['mean', 'std']
    })
    
    html = """
    <html>
    <head>
        <title>Reporte de Métricas NS-3 - Análisis Mejorado</title>
        <style>
            body { font-family: 'Segoe UI', Arial, sans-serif; margin: 30px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); }
            .container { background: white; border-radius: 10px; padding: 30px; box-shadow: 0 10px 40px rgba(0,0,0,0.3); }
            h1 { color: #2c3e50; text-align: center; border-bottom: 3px solid #3498db; padding-bottom: 15px; }
            h2 { color: #34495e; margin-top: 30px; }
            table { border-collapse: collapse; width: 100%; margin: 20px 0; background: white; }
            th { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 12px; text-align: left; }
            td { padding: 10px; border-bottom: 1px solid #ddd; }
            tr:hover { background: #f5f5f5; }
            .metric-excellent { color: #27ae60; font-weight: bold; }
            .metric-good { color: #2980b9; font-weight: bold; }
            .metric-warning { color: #f39c12; font-weight: bold; }
            .metric-bad { color: #e74c3c; font-weight: bold; }
            .summary-box { background: #ecf0f1; padding: 15px; margin: 15px 0; border-left: 4px solid #3498db; border-radius: 5px; }
            .highlight { background: #fffacd; padding: 2px 5px; border-radius: 3px; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>Reporte Completo de Análisis NS-3</h1>
            
            <div class="summary-box">
                <h3>Resumen Ejecutivo</h3>
                <p><strong>Total de experimentos:</strong> """ + str(len(df)) + """</p>
                <p><strong>Configuraciones de drones:</strong> """ + str(sorted(df['Num_Drones'].unique())) + """</p>
                <p><strong>Repeticiones por configuración:</strong> """ + str(df['Rep'].nunique()) + """</p>
                <p><strong>PDR promedio general:</strong> <span class="highlight">""" + f"{df['PDR_%'].mean():.2f}%" + """</span></p>
                <p><strong>Latencia promedio general:</strong> <span class="highlight">""" + f"{df['Lat_Prom_s'].mean():.2f}s" + """</span></p>
            </div>
            
            <h2>Estadísticas Detalladas</h2>
    """
    
    html += stats.to_html(float_format='%.3f')
    
    html += """
            <h2>Observaciones Clave</h2>
            <div class="summary-box">
                <ul>
                    <li>Los datos muestran el rendimiento real de la red AODV ad hoc</li>
                    <li>Un PDR bajo indica problemas de ruteo que requieren optimización</li>
                    <li>La latencia alta puede deberse a store-carry-forward</li>
                    <li>Aumentar drones puede mejorar cobertura pero también congestión</li>
                </ul>
            </div>
            
            <h2>Recomendaciones</h2>
            <div class="summary-box">
                <ol>
                    <li>Ajustar parámetros AODV (RREQ timeout, packet buffer)</li>
                    <li>Optimizar movilidad de drones para mejor cobertura</li>
                    <li>Considerar protocolos híbridos (AODV + geográfico)</li>
                    <li>Aumentar potencia de transmisión en condiciones adversas</li>
                </ol>
            </div>
        </div>
    </body>
    </html>
    """
    
    with open('reporte_mejorado.html', 'w', encoding='utf-8') as f:
        f.write(html)
    
    print("Reporte HTML guardado: reporte_mejorado.html")

def main():
    print("\n" + "="*70)
    print("  ANÁLISIS MEJORADO - Visualización de Datos Reales")
    print("="*70 + "\n")
    
    csv_file = 'metricas_simulacion.csv'
    df = load_metrics(csv_file)
    
    if len(df) == 0:
        print("No hay datos válidos para analizar")
        sys.exit(1)
    
    print("\nGenerando visualizaciones...\n")
    plot_comprehensive_analysis(df)
    generate_html_report(df)
    
    df.to_csv('metricas_filtradas.csv', index=False)
    print("CSV filtrado guardado: metricas_filtradas.csv")
    
    print("\n" + "="*70)
    print("ANÁLISIS COMPLETADO")
    print("="*70)
    print("\nArchivos generados:")
    print("   analisis_completo_mejorado.png")
    print("   reporte_mejorado.html")
    print("   metricas_filtradas.csv")
    print("\nLos gráficos muestran los datos reales de la simulación\n")

if __name__ == "__main__":
    main()
