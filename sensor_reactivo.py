import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime
import openpyxl  # Librería para trabajar con Excel ola
import math  # Para cálculos estadísticos

# Variables globales
ser = None            # Conexión del sensor (COM12 online)
arduino = None        # Conexión del Arduino (COM8)
running = False
continuous_mode = False
workbook = None
sheet = None
file_name = None
la_values = []  # Lista para almacenar los valores LA
session_duration = 0  # Duración de la sesión en segundos
measurement_interval = 3  # Intervalo de tiempo entre mediciones en segundos

def connect_serial():
    global ser, running
    port = port_var.get()  # Se usará el puerto del sensor (COM12)
    baud_rate = int(baud_var.get())
    data_bits = int(data_bits_var.get())
    parity = parity_var.get()
    stop_bits = float(stop_bits_var.get())

    parity_dict = {'None': serial.PARITY_NONE, 'Even': serial.PARITY_EVEN, 'Odd': serial.PARITY_ODD}

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud_rate,
            bytesize=data_bits,
            parity=parity_dict[parity],
            stopbits=stop_bits,
            timeout=1
        )
        running = True
        messagebox.showinfo("Conexión", f"Conectado al sensor en {port} con éxito.")
        threading.Thread(target=read_data, daemon=True).start()
    except serial.SerialException as e:
        messagebox.showerror("Error de conexión", f"No se pudo conectar al sensor: {e}")

def connect_arduino():
    """Conecta al Arduino en COM8."""
    global arduino
    try:
        arduino = serial.Serial("COM8", 9600, timeout=1)
        print("Arduino conectado en COM8.")
    except Exception as e:
        print("Error al conectar Arduino en COM8:", e)

def continuous_send():
    global continuous_mode
    while continuous_mode:
        send_data()
        time.sleep(measurement_interval)  # Usar el intervalo de tiempo definido

def send_data():
    if ser and ser.is_open:
        data = "0103000e0001e5c9"
        try:
            ser.write(bytes.fromhex(data))
        except ValueError:
            messagebox.showerror("Error de formato", "El formato de los datos es incorrecto.")
    else:
        messagebox.showwarning("Sin conexión", "Primero conecta el puerto serial del sensor.")

def read_data():
    global la_values
    while running:
        if ser and ser.in_waiting > 0:
            raw_data = ser.read(5)  # Leer los primeros 5 bytes

            if len(raw_data) >= 5:
                fourth_fifth_bytes = raw_data[3:5]
                decimal_value = int.from_bytes(fourth_fifth_bytes, byteorder='big')
                multiplied_value = decimal_value * 0.1

                # Ignorar valores fuera del rango 30-120 dB
                if 30 <= multiplied_value <= 120:
                    la_values.append(multiplied_value)
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    terminal_display.insert(tk.END, f"[{timestamp}] Ruido en dB: {multiplied_value:.2f}\n")
                    terminal_display.see(tk.END)
                    save_to_excel(timestamp, multiplied_value)
                    # Enviar el valor al Arduino (COM8)
                    try:
                        if arduino and arduino.is_open:
                            # Se envía como entero seguido de salto de línea
                            arduino.write(f"{int(multiplied_value)}\n".encode())
                    except Exception as e:
                        print(f"Error al enviar dato al Arduino: {e}")
        time.sleep(0.1)

def save_to_excel(timestamp, value):
    global workbook, sheet, file_name

    try:
        if workbook is None:
            messagebox.showerror("Error", "El archivo Excel no está inicializado.")
            return

        sheet.append([timestamp, value])
        sheet.column_dimensions['A'].width = 20
        workbook.save(file_name)
    except Exception as e:
        messagebox.showerror("Error al guardar en Excel", f"No se pudo guardar la medición: {e}")

def calculate_statistics():
    global la_values, workbook, sheet

    if not la_values:
        messagebox.showwarning("Sin datos", "No se registraron valores para calcular las estadísticas.")
        return

    # Calcular las estadísticas
    laf_max = max(la_values)
    laf_min = min(la_values)
    lafeq = 10 * math.log10(sum(10**(value/10) for value in la_values) / len(la_values))
    la_values_sorted = sorted(la_values)
    laf10 = la_values_sorted[int(0.1 * len(la_values))]
    laf90 = la_values_sorted[int(0.9 * len(la_values))]

    # Guardar los valores calculados en Excel
    next_row = sheet.max_row + 2  # Dejar una fila en blanco antes de las estadísticas
    sheet[f"C{next_row}"] = "Estadísticas"
    sheet[f"C{next_row + 1}"] = "LAFmax"
    sheet[f"D{next_row + 1}"] = laf_max
    sheet[f"C{next_row + 2}"] = "LAFmin"
    sheet[f"D{next_row + 2}"] = laf_min
    sheet[f"C{next_row + 3}"] = "LAF10"
    sheet[f"D{next_row + 3}"] = laf10
    sheet[f"C{next_row + 4}"] = "LAF90"
    sheet[f"D{next_row + 4}"] = laf90
    sheet[f"C{next_row + 5}"] = "LAFeq"
    sheet[f"D{next_row + 5}"] = lafeq

    # Guardar el archivo Excel
    workbook.save(file_name)
    messagebox.showinfo("Estadísticas guardadas", "Se han calculado y guardado las estadísticas en el archivo Excel.")

def stop_after_duration():
    global running, continuous_mode
    time.sleep(session_duration)  # Esperar el tiempo de duración de la sesión
    if continuous_mode:
        toggle_continuous_mode()  # Detener la medición
        if la_values:  # Solo calcular estadísticas si hay datos
            calculate_statistics()  # Calcular estadísticas al finalizar la sesión

def toggle_continuous_mode():
    global continuous_mode, workbook, sheet, file_name, la_values, session_duration, measurement_interval
    if continuous_mode:
        continuous_mode = False
        start_button.config(text="Iniciar Envío Continuo")
        if workbook:
            workbook.save(file_name)
            workbook = None
            sheet = None
            file_name = None
            la_values = []  # Limpiar la lista de valores solo después de guardar
            messagebox.showinfo("Archivo guardado", f"Las mediciones se guardaron en {file_name}")
    else:
        continuous_mode = True
        start_button.config(text="Detener Envío Continuo")
        file_name = f"mediciones_ruido_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.xlsx"
        workbook = openpyxl.Workbook()
        sheet = workbook.active
        sheet.append(["Fecha y Hora", "Ruido (dB)"])
        sheet.column_dimensions['A'].width = 20

        # Obtener el intervalo de tiempo y la duración de la sesión
        try:
            measurement_interval = int(interval_var.get())
            session_duration = int(duration_var.get())
        except ValueError:
            messagebox.showerror("Error", "El intervalo de tiempo y la duración deben ser números válidos.")
            return

        # Iniciar el envío continuo
        threading.Thread(target=continuous_send, daemon=True).start()
        # Iniciar el temporizador para detener la medición después del tiempo definido
        threading.Thread(target=stop_after_duration, daemon=True).start()

def close_serial():
    global running, continuous_mode, workbook, sheet, file_name, arduino
    running = False
    continuous_mode = False
    if ser and ser.is_open:
        ser.close()
        messagebox.showinfo("Desconexión", "Puerto serial del sensor cerrado.")
    if arduino and arduino.is_open:
        arduino.close()
        print("Conexión con Arduino cerrada.")
    if workbook:
        workbook.save(file_name)
        workbook = None
        sheet = None
        file_name = None

def get_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

# Interfaz gráfica con Tkinter
root = tk.Tk()
root.title("Terminal Serial")
root.geometry("500x700")

# Variables de configuración para el sensor
port_var = tk.StringVar(value=get_ports()[0] if get_ports() else "")
baud_var = tk.StringVar(value="9600")
data_bits_var = tk.StringVar(value="8")
parity_var = tk.StringVar(value="None")
stop_bits_var = tk.StringVar(value="1")
interval_var = tk.StringVar(value="3")  # Intervalo de tiempo entre mediciones
duration_var = tk.StringVar(value="60")  # Duración de la sesión en segundos

ttk.Label(root, text="Puerto Sensor:").grid(row=0, column=0, padx=5, pady=5)
ttk.Combobox(root, textvariable=port_var, values=get_ports()).grid(row=0, column=1, padx=5, pady=5)

ttk.Label(root, text="Baud Rate:").grid(row=1, column=0, padx=5, pady=5)
ttk.Combobox(root, textvariable=baud_var, values=["9600", "115200", "4800", "2400"]).grid(row=1, column=1, padx=5, pady=5)

ttk.Label(root, text="Data Bits:").grid(row=2, column=0, padx=5, pady=5)
ttk.Combobox(root, textvariable=data_bits_var, values=["5", "6", "7", "8"]).grid(row=2, column=1, padx=5, pady=5)

ttk.Label(root, text="Parity:").grid(row=3, column=0, padx=5, pady=5)
ttk.Combobox(root, textvariable=parity_var, values=["None", "Even", "Odd"]).grid(row=3, column=1, padx=5, pady=5)

ttk.Label(root, text="Stop Bits:").grid(row=4, column=0, padx=5, pady=5)
ttk.Combobox(root, textvariable=stop_bits_var, values=["1", "1.5", "2"]).grid(row=4, column=1, padx=5, pady=5)

# Campo de entrada para el intervalo de tiempo entre mediciones
ttk.Label(root, text="Intervalo de tiempo (segundos):").grid(row=5, column=0, padx=5, pady=5)
ttk.Entry(root, textvariable=interval_var).grid(row=5, column=1, padx=5, pady=5)

# Campo de entrada para la duración de la sesión
ttk.Label(root, text="Duración de la sesión (segundos):").grid(row=6, column=0, padx=5, pady=5)
ttk.Entry(root, textvariable=duration_var).grid(row=6, column=1, padx=5, pady=5)

ttk.Button(root, text="Conectar Sensor", command=connect_serial).grid(row=7, column=0, columnspan=2, pady=5)
start_button = ttk.Button(root, text="Iniciar Envío Continuo", command=toggle_continuous_mode)
start_button.grid(row=8, column=0, columnspan=2, pady=5)
ttk.Button(root, text="Cerrar", command=close_serial).grid(row=9, column=0, columnspan=2, pady=5)

ttk.Label(root, text="Terminal de Lectura:").grid(row=11, column=0, columnspan=2, pady=5)
terminal_display = scrolledtext.ScrolledText(root, width=50, height=10, state='normal')
terminal_display.grid(row=12, column=0, columnspan=2, padx=10, pady=5)

# Conectar al Arduino en COM8 al iniciar la aplicación
connect_arduino()

root.mainloop()