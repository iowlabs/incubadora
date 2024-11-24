import csv
import matplotlib.pyplot as plt

def procesar_csv(nombre_archivo):
	valores = []
	tiempos = []
	tiempo_inicial = 0

	# Leer el archivo CSV
	with open(nombre_archivo, mode='r') as archivo_csv:
		lector = csv.reader(archivo_csv)
		#next(lector)  # Omitir el encabezado si existe

		for fila in lector:
			try:
				# Suponemos que la tercera columna contiene datos numéricos
				valor = float(fila[2])
				tiempo = float(fila[0])
				valores.append(valor)
				if len(tiempos) > 1:
					tiempo_inicial = tiempos[0]
				tiempos.append(tiempo-tiempo_inicial)
			except (IndexError, ValueError):
				print(f"Error al procesar la fila: {fila}")

	# Calcular el acumulado y la media
	acumulado = sum(valores)
	delta_tiempo = tiempos[-1]-tiempos[0]
	media = acumulado / len(valores) if valores else 0
	return  valores, tiempos, acumulado, media

def generar_grafico(valores,tiempos, media):
	plt.figure(figsize=(10, 6))
	plt.plot(tiempos, valores, label='Corriente en mA')
	plt.axhline(y=media, color='r', linestyle='--', label=f'Media = {media:.2f}')
	plt.title('BAM en operación')
	plt.xlabel('tiempo [ms]')
	plt.ylabel('Corriente[mA]')
	plt.legend()
	plt.grid(True)
	plt.tight_layout()
	plt.show()


# Ejemplo de uso
nombre_archivo = "2024-11-20_06-53.csv"  # Reemplaza con la ruta de tu archivo
valores,tiempos,acumulado, media = procesar_csv(nombre_archivo)

print(f"Valor acumulado de la tercera columna: {(acumulado*0.25/1000.0)} A")
#print(f"Valor acumulado de la tercera columna: { (acumulado * 0.25)/3600 } mAh")
print(f"Media de la tercera columna: {media/1000.0} [A]")
print(f"tiempo maximo : {tiempos[-1]/1000.0} [s]")
print(f"carga  = {(acumulado*0.25)/(1000.0*3600.0)} [Ah]")
# Generar el gráfico
generar_grafico(valores,tiempos, media)
