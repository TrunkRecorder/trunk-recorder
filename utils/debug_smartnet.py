#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: SmartNet Debug
# GNU Radio version: 3.10.12.0

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio import blocks
import pmt
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
import sip
import threading



class debug_smartnet(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "SmartNet Debug", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("SmartNet Debug")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "debug_smartnet")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 48000

        ##################################################
        # Blocks
        ##################################################

        self.throttle_sym = blocks.throttle(gr.sizeof_float*1, samp_rate,True)
        self.throttle_in = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate,True)
        self.throttle_demod = blocks.throttle(gr.sizeof_float*1, samp_rate,True)
        self.src_sym = blocks.file_source(gr.sizeof_float*1, '/Users/luke/Projects/TrunkRecorder/build/smartnet_debug_sym.float', True, 0, 0)
        self.src_sym.set_begin_tag(pmt.PMT_NIL)
        self.src_in = blocks.file_source(gr.sizeof_gr_complex*1, '/Users/luke/Projects/TrunkRecorder/build/smartnet_debug_in.cfile', True, 0, 0)
        self.src_in.set_begin_tag(pmt.PMT_NIL)
        self.src_demod = blocks.file_source(gr.sizeof_float*1, '/Users/luke/Projects/TrunkRecorder/build/smartnet_debug_demod.float', True, 0, 0)
        self.src_demod.set_begin_tag(pmt.PMT_NIL)
        self.sink_sym = qtgui.time_sink_f(
            1024, #size
            samp_rate, #samp_rate
            'Symbol Filter (Float)', #name
            1, #number of inputs
            None # parent
        )
        self.sink_sym.set_update_time(0.10)
        self.sink_sym.set_y_axis(-1, 1)

        self.sink_sym.set_y_label('Amplitude', "")

        self.sink_sym.enable_tags(True)
        self.sink_sym.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.sink_sym.enable_autoscale(True)
        self.sink_sym.enable_grid(False)
        self.sink_sym.enable_axis_labels(True)
        self.sink_sym.enable_control_panel(False)
        self.sink_sym.enable_stem_plot(False)


        labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(1):
            if len(labels[i]) == 0:
                self.sink_sym.set_line_label(i, "Data {0}".format(i))
            else:
                self.sink_sym.set_line_label(i, labels[i])
            self.sink_sym.set_line_width(i, widths[i])
            self.sink_sym.set_line_color(i, colors[i])
            self.sink_sym.set_line_style(i, styles[i])
            self.sink_sym.set_line_marker(i, markers[i])
            self.sink_sym.set_line_alpha(i, alphas[i])

        self._sink_sym_win = sip.wrapinstance(self.sink_sym.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._sink_sym_win)
        self.sink_in = qtgui.freq_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            'Input (Complex)', #name
            1,
            None # parent
        )
        self.sink_in.set_update_time(0.10)
        self.sink_in.set_y_axis((-140), 10)
        self.sink_in.set_y_label('Relative Gain', 'dB')
        self.sink_in.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.sink_in.enable_autoscale(False)
        self.sink_in.enable_grid(False)
        self.sink_in.set_fft_average(1.0)
        self.sink_in.enable_axis_labels(True)
        self.sink_in.enable_control_panel(False)
        self.sink_in.set_fft_window_normalized(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.sink_in.set_line_label(i, "Data {0}".format(i))
            else:
                self.sink_in.set_line_label(i, labels[i])
            self.sink_in.set_line_width(i, widths[i])
            self.sink_in.set_line_color(i, colors[i])
            self.sink_in.set_line_alpha(i, alphas[i])

        self._sink_in_win = sip.wrapinstance(self.sink_in.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._sink_in_win)
        self.sink_demod = qtgui.time_sink_f(
            1024, #size
            samp_rate, #samp_rate
            'Demod (Float)', #name
            1, #number of inputs
            None # parent
        )
        self.sink_demod.set_update_time(0.10)
        self.sink_demod.set_y_axis(-1, 1)

        self.sink_demod.set_y_label('Amplitude', "")

        self.sink_demod.enable_tags(True)
        self.sink_demod.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.sink_demod.enable_autoscale(True)
        self.sink_demod.enable_grid(False)
        self.sink_demod.enable_axis_labels(True)
        self.sink_demod.enable_control_panel(False)
        self.sink_demod.enable_stem_plot(False)


        labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(1):
            if len(labels[i]) == 0:
                self.sink_demod.set_line_label(i, "Data {0}".format(i))
            else:
                self.sink_demod.set_line_label(i, labels[i])
            self.sink_demod.set_line_width(i, widths[i])
            self.sink_demod.set_line_color(i, colors[i])
            self.sink_demod.set_line_style(i, styles[i])
            self.sink_demod.set_line_marker(i, markers[i])
            self.sink_demod.set_line_alpha(i, alphas[i])

        self._sink_demod_win = sip.wrapinstance(self.sink_demod.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._sink_demod_win)
        self.qtgui_waterfall_sink_x_0 = qtgui.waterfall_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            "", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_waterfall_sink_x_0.set_update_time(0.10)
        self.qtgui_waterfall_sink_x_0.enable_grid(False)
        self.qtgui_waterfall_sink_x_0.enable_axis_labels(True)



        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        colors = [0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_waterfall_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_waterfall_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_waterfall_sink_x_0.set_color_map(i, colors[i])
            self.qtgui_waterfall_sink_x_0.set_line_alpha(i, alphas[i])

        self.qtgui_waterfall_sink_x_0.set_intensity_range(-140, 10)

        self._qtgui_waterfall_sink_x_0_win = sip.wrapinstance(self.qtgui_waterfall_sink_x_0.qwidget(), Qt.QWidget)

        self.top_layout.addWidget(self._qtgui_waterfall_sink_x_0_win)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.src_demod, 0), (self.throttle_demod, 0))
        self.connect((self.src_in, 0), (self.throttle_in, 0))
        self.connect((self.src_sym, 0), (self.throttle_sym, 0))
        self.connect((self.throttle_demod, 0), (self.sink_demod, 0))
        self.connect((self.throttle_in, 0), (self.qtgui_waterfall_sink_x_0, 0))
        self.connect((self.throttle_in, 0), (self.sink_in, 0))
        self.connect((self.throttle_sym, 0), (self.sink_sym, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "debug_smartnet")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.qtgui_waterfall_sink_x_0.set_frequency_range(0, self.samp_rate)
        self.sink_demod.set_samp_rate(self.samp_rate)
        self.sink_in.set_frequency_range(0, self.samp_rate)
        self.sink_sym.set_samp_rate(self.samp_rate)
        self.throttle_demod.set_sample_rate(self.samp_rate)
        self.throttle_in.set_sample_rate(self.samp_rate)
        self.throttle_sym.set_sample_rate(self.samp_rate)




def main(top_block_cls=debug_smartnet, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()
    tb.flowgraph_started.set()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
