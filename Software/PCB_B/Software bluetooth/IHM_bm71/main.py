#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Terminal Série Bluetooth Low Energy (BLE) pour BM71 Microchip
- Mode Transparent UART activé
- Compatible firmware BM71 v1.06+
"""

import asyncio
import sys
from bleak import BleakScanner, BleakClient
import threading
try:
    import readline                  # Linux / macOS
except ImportError:
    import pyreadline as readline    # Windows

# UUIDs du Transparent UART Service (TUS) pour BM71
TUS_SERVICE_UUID = "49535343-FE7D-4AE5-8FA9-9FAFD205E455"  # Service principal
TUS_RX_UUID = "49535343-6D99-4B8A-98E5-9FAFD205E455"   # TX du module → RX PC (notifications)
TUS_TX_UUID = "49535343-1E4D-4BD9-BA61-23C647249616"   # RX du module ← TX PC (write)

client = None
stop_event = threading.Event()

def handle_rx(_: int, data: bytearray):
    """Réception des données du BM71 → affichage"""
    if data:
        print(data.decode('utf-8', errors='replace'), end='', flush=True)

async def connect_to_bm71(device_address=None):
    global client

    print("Scan des périphériques BLE... Recherche BM71 avec Transparent UART Service...")

    if device_address:
        device = await BleakScanner.find_device_by_address(device_address, timeout=10)
    else:
        # Filtre pour BM71 : nom contenant "BM71" ou service TUS
        devices = await BleakScanner.find_device_by_filter(
            lambda d, ad: (
                "BM71" in (d.name or "") or
                TUS_SERVICE_UUID.lower() in [s.lower() for s in ad.service_uuids] if ad.service_uuids else False
            ),
            timeout=10
        )
        if not devices:
            print("Aucun BM71 trouvé. Vérifie que le module est en mode advertising (LED clignote ?).")
            return None

        print("\nBM71 trouvés :")
        for i, d in enumerate(devices):
            print(f"  [{i}] {d.name or 'BM71'} → {d.address}")
        
        choix = input("\nChoisis un numéro (ou Entrée pour le premier) : ")
        try:
            device = devices[int(choix)] if choix.strip() else devices[0]
        except:
            device = devices[0]

    print(f"\nConnexion au BM71 ({device.address})...")
    client = BleakClient(device.address)
    await client.connect()
    
    # Vérifie le service TUS
    services = await client.get_services()
    service = services.get_service(TUS_SERVICE_UUID)
    if not service:
        print("Service Transparent UART non trouvé ! Vérifie la config du BM71 (mode Auto/Manuel).")
        await client.disconnect()
        return None

    print("Connecté au BM71 ! Terminal série prêt.")
    print("En mode Transparent UART : tape du texte + Entrée pour envoyer.")
    print("Réception en temps réel. Ctrl+C pour quitter.\n")

    # Active notifications sur RX (réception)
    await client.start_notify(TUS_RX_UUID, handle_rx)

    return client

async def terminal_loop():
    global client
    while not stop_event.is_set():
        try:
            if not client or not client.is_connected:
                break

            ligne = await asyncio.get_event_loop().run_in_executor(None, input, "")
            if ligne.lower() in ["exit", "quit", "q"]:
                break
            if ligne.strip():
                # Envoi au BM71 (sans \r\n auto, car transparent – ajoute si besoin)
                data_to_send = ligne.encode('utf-8')
                await client.write_gatt_char(TUS_TX_UUID, data_to_send)
        except EOFError:
            break
        except KeyboardInterrupt:
            print("\nDéconnexion...")
            break
        except Exception as e:
            print(f"\nErreur : {e}")
            break

async def main():
    # Option : adresse MAC du BM71 pour connexion directe
    TARGET_ADDRESS = None  # Ex: "AA:BB:CC:DD:EE:FF"

    client = await connect_to_bm71(TARGET_ADDRESS)
    if client:
        try:
            await terminal_loop()
        finally:
            if client.is_connected:
                await client.start_notify(TUS_RX_UUID)  # Arrête notifications
                await client.disconnect()
            print("Déconnecté du BM71. À plus !")

if __name__ == "__main__":
    print("""
╔═══════════════════════════════════════════════════╗
║   Terminal Série BLE pour BM71 (Microchip)        ║
║             Mode Transparent UART                 ║
╚═══════════════════════════════════════════════════╝
    """)

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nArrêt.")