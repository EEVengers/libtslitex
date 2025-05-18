#!/usr/bin/env python3
""" SPDX-License-Identifier: BSD-2-Clause
@file tsControl.py

@brief Thunderscope Control GUI

@details
A graphical user interface (GUI) application for controlling and monitoring a Thunderscope device.
This script provides functionality to connect to a Thunderscope, configure its channels, and view its status.
It uses the Tkinter library for the GUI and interacts with the Thunderscope through the tslitex library.

Features:
- Device connection and disconnection management.
- Channel configuration with adjustable parameters.
- Real-time status monitoring of the Thunderscope device.

@copyright (C) 2025 / Nate Meyer  / nate.devel@gmail.com
"""

import tslitex
import tkinter as tk
from tkinter import ttk

class TsDelegate:
    """
    A delegate class to handle events and actions for the TsControl.
    """

    def __init__(self, parent):
        self.parent = parent
        self.ts_inst = None
        self.dev_list = []

    def on_channel_apply(self, channel, params):
        print(f"Applying settings for Channel {channel}: {params}")
        if self.ts_inst:
            self.ts_inst.channel[channel-1].ManualCtrl(params)

    def get_devices(self):
        print("Retrieving devices...")
        # Return a list of devices (this can be replaced with actual logic to fetch devices)
        self.dev_list = []
        idx = 0
        while True:
            res, dev_info = tslitex.ThunderscopeListDevs(idx)
            if res == -1:    
                break
            else:
                print(f"Found device {idx}: {dev_info}")
                self.dev_list.append(f"{idx}: Thunderscope {dev_info['device_path'].decode('utf-8')}")
                idx += 1
        return self.dev_list

    def connect_device(self, device):
        # Handle the connect event
        print(f"Connecting to device: {device}")
        self.ts_inst = tslitex.Thunderscope(dev_idx=device)
        self.update_status(f"Connected: Thunderscope {device}")
        # Change the connect button to say disconnect
        connect_button = self.parent.nametowidget('top_frame.connect_button')
        connect_button.config(text="Disconnect", command=self.on_close)

        # Reset fields for all ChannelCtrl instances
        for tab in self.parent.nametowidget('!notebook').winfo_children():
            if isinstance(tab, ChannelCtrl):
                tab.reset_fields()

    def on_close(self):
        # Handle the window close event
        if self.ts_inst:
            print("Deleting ts_inst...")
            del self.ts_inst
            self.ts_inst = None
        # Change the connect button to say connect
        connect_button = self.parent.nametowidget('top_frame.connect_button')
        connect_button.config(text="Connect", command=lambda: self.connect_device(self.parent.nametowidget('top_frame.device_combobox').current()))
        self.update_status("Disconnected")

    def update_status(self, message):
        status_bar = self.parent.nametowidget('status_bar')
        status_bar.config(text=message)


class ChannelCtrl(ttk.Frame):
    """
    A class to create a channel control interface using Tkinter.
    """

    def __init__(self, parent, channel, on_channel_apply_callback):
        super().__init__(parent)
        self.channel = channel
        self.on_channel_apply_callback = on_channel_apply_callback
        self.enable = tk.BooleanVar(value=False)
        self.atten = tk.BooleanVar(value=False)
        self.dc_couple = tk.BooleanVar(value=False)
        self.termination = tk.BooleanVar(value=False)
        self.pga_high_gain = tk.BooleanVar(value=False)
        self.pga_bw = tk.IntVar(value=6)
        self.pga_atten = tk.IntVar(value=10)
        self.trim_dac = tk.IntVar(value=2048)
        self.trim_pot = tk.IntVar(value=0x40)
        self.create_widgets()

    def create_widgets(self):
        # Create a frame
        frame = ttk.Frame(self, padding=10)
        frame.pack(expand=True, fill="both")

        # Add Checkboxes
        tk.Checkbutton(frame, text="Attenuation", variable=self.atten).grid(row=2, column=0, sticky="w", pady=5)
        tk.Checkbutton(frame, text="Termination", variable=self.termination).grid(row=3, column=0, sticky="w", pady=5)
        tk.Checkbutton(frame, text="DC Coupling", variable=self.dc_couple).grid(row=4, column=0, sticky="w", pady=5)
        tk.Checkbutton(frame, text="PGA High-Gain", variable=self.pga_high_gain).grid(row=5, column=0, sticky="w", pady=5)

        # Add Text Boxes with Range Labels
        ttk.Label(frame, text="Trim DAC:").grid(row=2, column=1, sticky="w", pady=5)
        trim_dac_entry = ttk.Entry(frame, textvariable=self.trim_dac, width=10)
        trim_dac_entry.grid(row=2, column=2, pady=5, padx=5)
        ttk.Label(frame, text="(Range: 0 - 4095)").grid(row=2, column=3, sticky="w", padx=5)

        ttk.Label(frame, text="Trim Pot:").grid(row=3, column=1, sticky="w", pady=5)
        trim_pot_entry = ttk.Entry(frame, textvariable=self.trim_pot, width=10)
        trim_pot_entry.grid(row=3, column=2, pady=5, padx=5)
        ttk.Label(frame, text="(Range: 0 - 255)").grid(row=3, column=3, sticky="w", padx=5)

        ttk.Label(frame, text="PGA Attenuation:").grid(row=4, column=1, sticky="w", pady=5)
        pga_atten_entry = ttk.Entry(frame, textvariable=self.pga_atten, width=10)
        pga_atten_entry.grid(row=4, column=2, pady=5, padx=5)
        ttk.Label(frame, text="(Range: 0 - 10)").grid(row=4, column=3, sticky="w", padx=5)

        ttk.Label(frame, text="PGA Bandwidth:").grid(row=5, column=1, sticky="w", pady=5)
        pga_bw_entry = ttk.Entry(frame, textvariable=self.pga_bw, width=10)
        pga_bw_entry.grid(row=5, column=2, pady=5, padx=5)
        ttk.Label(frame, text="(Range: 0 - 6)").grid(row=5, column=3, sticky="w", padx=5)

        # Add a Button
        ttk.Button(frame, text="Apply", command=self.submit).grid(row=6, column=0, pady=10)

    def submit(self):
        # Validate and adjust entry values
        dac = int(self.trim_dac.get())
        if dac < 0:
            dac = 0
            self.trim_dac.set(dac)
        elif dac > 4095:
            dac = 4095
            self.trim_dac.set(dac)

        dpot = int(self.trim_pot.get())
        if dpot < 0:
            dpot = 0
            self.trim_pot.set(dpot)
        elif dpot > 255:
            dpot = 255
            self.trim_pot.set(dpot)

        pga_atten = int(self.pga_atten.get())
        if pga_atten < 0:
            pga_atten = 0
            self.pga_atten.set(pga_atten)
        elif pga_atten > 10:
            pga_atten = 10
            self.pga_atten.set(pga_atten)

        pga_bw = int(self.pga_bw.get())
        if pga_bw < 0:
            pga_bw = 0
            self.pga_bw.set(pga_bw)
        elif pga_bw > 6:
            pga_bw = 6
            self.pga_bw.set(pga_bw)

        # Collect parameters from the checkbuttons and entry fields
        params = {
            "atten": self.atten.get(),
            "term": self.termination.get(),
            "dc_couple": self.dc_couple.get(),
            "dac": dac,
            "dpot": dpot,
            "pga_high_gain": self.pga_high_gain.get(),
            "pga_atten": pga_atten,
            "pga_bw": pga_bw
        }

        # Call the provided callback with the channel and parameters
        if self.on_channel_apply_callback:
            self.on_channel_apply_callback(self.channel, params)

    def reset_fields(self):
        # Reset checkboxes
        self.atten.set(False)
        self.dc_couple.set(False)
        self.termination.set(False)
        self.pga_high_gain.set(False)

        # Reset entry fields to their default values
        self.trim_dac.set(2048)
        self.trim_pot.set(0x40)
        self.pga_atten.set(10)
        self.pga_bw.set(6)


# Create the main application window
root = tk.Tk()
root.title("Thunderscope Channel Tester")
root.geometry("600x450")  # Set the window size

# Create an instance of TsDelegate
delegate = TsDelegate(root)
root.protocol("WM_DELETE_WINDOW", lambda: (delegate.on_close(), root.destroy()))


# Create a frame for the drop-down menu and button
top_frame = ttk.Frame(root, padding=10, name="top_frame")
top_frame.pack(side="top", fill="x")

# Add a drop-down menu (Combobox) to the top frame
ttk.Label(top_frame, text="Select Device:").pack(side="left", padx=5)
device_combobox = ttk.Combobox(top_frame, values=delegate.get_devices(), state="readonly", name="device_combobox")
device_combobox.pack(side="left", padx=5)

# Set the width of the combobox to increase its size
device_combobox.config(width=40)

# Add a callback to refresh the device list when the dropdown menu is clicked
def refresh_device_list(event):
    device_combobox["values"] = delegate.get_devices()

device_combobox.bind("<Button-1>", refresh_device_list)

# Add a "Connect" button to the top frame
connect_button = ttk.Button(
    top_frame,
    text="Connect",
    command=lambda: delegate.connect_device(device_combobox.current()),
    name="connect_button"
)
connect_button.pack(side="left", padx=5)


# Create a style for the notebook
style = ttk.Style()
style.configure("TNotebook.Tab", padding=[10, 5])  # Add padding to tabs

# Update the notebook style to position tabs on the left side
style.configure("TNotebook", tabposition="wn")
style.configure("TNotebook", expand=True)  # Configure the notebook tabs to evenly fill the width of the window
style.map("TNotebook.Tab", expand=[("selected", 1), ("!selected", 1)])  # Stretch tabs evenly


# Create a Notebook widget (for tabs)
notebook = ttk.Notebook(root, style="TNotebook")


# Modify the status tab to display labels and number fields for each key in tsScopeState_t
def create_status_tab(notebook, delegate):
    status_tab = ttk.Frame(notebook)

    # Create a dictionary to hold labels for each key in tsScopeState_t
    status_labels = {}

    # Define the keys from tsScopeState_t
    keys = [
        "adc_sample_rate", "adc_sample_bits", "adc_sample_resolution", "adc_lost_buffer_count",
        "flags", "adc_state", "power_state", "pll_state", "afe_state",
        "sys_health.temp_c", "sys_health.vcc_int", "sys_health.vcc_aux",
        "sys_health.vcc_bram", "sys_health.frontend_power_good", "sys_health.acq_power_good"
    ]

    # Create labels and fields for each key
    for idx, key in enumerate(keys):
        ttk.Label(status_tab, text=key + ":").grid(row=idx, column=0, sticky="w", padx=10, pady=2)
        status_labels[key] = ttk.Label(status_tab, text="N/A")
        status_labels[key].grid(row=idx, column=1, sticky="w", padx=10, pady=2)

    def update_status():
        if delegate.ts_inst:
            status = delegate.ts_inst.Status()
            for key in keys:
                # Handle nested keys like sys_health.temp_c
                value = status
                for part in key.split('.'):
                    value = value.get(part, "N/A")
                status_labels[key].config(text=value)
        else:
            for key in keys:
                status_labels[key].config(text="N/A")

    # Periodically update the status only when the tab is selected
    def periodic_update():
        if notebook.index(notebook.select()) == 0:  # Check if tab0 is selected
            update_status()
            status_tab.after(1000, periodic_update)

    # Bind events to start and stop updates
    def on_tab_changed(event):
        if notebook.index(notebook.select()) == 0:  # Tab0 selected
            periodic_update()

    notebook.bind("<<NotebookTabChanged>>", on_tab_changed)
    return status_tab

# Add the status tab to the notebook
tab0 = create_status_tab(notebook, delegate)
notebook.add(tab0, text="Status")


# Create frames for each tab
tab1 = ChannelCtrl(notebook, 1, delegate.on_channel_apply)
tab2 = ChannelCtrl(notebook, 2, delegate.on_channel_apply)
tab3 = ChannelCtrl(notebook, 3, delegate.on_channel_apply)
tab4 = ChannelCtrl(notebook, 4, delegate.on_channel_apply)

# Add tabs to the notebook
notebook.add(tab1, text="Chan 1")
notebook.add(tab2, text="Chan 2")
notebook.add(tab3, text="Chan 3")
notebook.add(tab4, text="Chan 4")

# Pack the notebook widget into the main window
notebook.pack(expand=True, fill="both")

# Add a status bar to the bottom of the window
status_bar = ttk.Label(root, text="Disconnected", relief="sunken", anchor="w", name="status_bar")
status_bar.pack(side="bottom", fill="x")

# Start the Tkinter event loop
root.mainloop()
