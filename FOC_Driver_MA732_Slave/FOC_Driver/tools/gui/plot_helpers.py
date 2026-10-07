# -*- coding: utf-8 -*-
from .deps import MODE_CURRENT, MODE_VELOCITY


class PlotHelperMixin:
    def _prune_buffers(self):
        if not self.cb_follow.isChecked():
            return
        if not self.t_buf and not self.t_ref:
            return
        win = float(self.spin_timewin.value())
        latest = max(self.t_buf[-1] if self.t_buf else float('-inf'),
                     self.t_ref[-1] if self.t_ref else float('-inf'))
        cutoff = latest - win
        while self.t_buf and self.t_buf[0] < cutoff:
            self.t_buf.popleft()
            self.pos_buf.popleft()
            self.spd_buf.popleft()
            self.cur_buf.popleft()
            self.tmp_buf.popleft()
        if self.t_ref:
            while self.t_ref and self.t_ref[0] < cutoff:
                self.t_ref.popleft()
                self.y_ref.popleft()

    def _decimate_minmax(self, xs, ys, max_points):
        n = len(xs)
        if n == 0 or n != len(ys):
            return ([], [])
        if n <= max_points:
            return (xs, ys)
        bins = max(1, max_points // 2)
        step = max(1, n // bins)
        x_out = []
        y_out = []
        for i in range(0, n, step):
            x_chunk = xs[i:i + step]
            y_chunk = ys[i:i + step]
            if not y_chunk:
                continue
            y_min = min(y_chunk)
            y_max = max(y_chunk)
            if y_min == y_max:
                x_out.append(x_chunk[len(x_chunk) // 2])
                y_out.append(y_min)
            else:
                i_min = y_chunk.index(y_min)
                i_max = y_chunk.index(y_max)
                if i_min <= i_max:
                    x_out.append(x_chunk[i_min]); y_out.append(y_min)
                    x_out.append(x_chunk[i_max]); y_out.append(y_max)
                else:
                    x_out.append(x_chunk[i_max]); y_out.append(y_max)
                    x_out.append(x_chunk[i_min]); y_out.append(y_min)
            if len(x_out) >= max_points:
                break
        if x_out and x_out[-1] != xs[-1]:
            if len(x_out) >= max_points:
                x_out.pop(0); y_out.pop(0)
            x_out.append(xs[-1]); y_out.append(ys[-1])
        return (x_out, y_out)

    def _sync_views(self):
        vb_rect = self.vb_pos.sceneBoundingRect()
        for vb in (self.vb_spd, self.vb_cur, self.vb_tmp, self.vb_ref):
            vb.setGeometry(vb_rect)
            vb.linkedViewChanged(self.vb_pos, vb.XAxis)

    def _on_fit(self):
        """X/Y 모두 한 번에 자동 맞춤."""
        if not self.t_buf and not self.t_ref:
            self._log("Auto Fit: 데이터가 아직 없습니다.")
            return

        if self.t_buf:
            t0 = self.t_buf[0]
            xs = [t - t0 for t in self.t_buf]
        else:
            t0 = self.t_ref[0]
            xs = [t - t0 for t in self.t_ref]

        if xs:
            x_min = min(xs)
            x_max = max(xs)
            if x_min == x_max:
                x_min -= 0.5
                x_max += 0.5
            self.vb_pos.setXRange(x_min, x_max, padding=0.02)

        self._on_fit_y()

    def _on_fit_y(self):
        """현재 X범위와 체크된 곡선 기준으로 Y만 자동 맞춤."""
        if not self.t_buf and not self.t_ref:
            self._log("Auto Y: 데이터가 아직 없습니다.")
            return

        (x_min, x_max), _ = self.vb_pos.viewRange()

        # 기준 시간(t0): 수신 데이터가 있으면 그걸, 없으면 ref 기준
        if self.t_buf:
            t0 = self.t_buf[0]
            xs = [t - t0 for t in self.t_buf]
        else:
            t0 = self.t_ref[0]
            xs = []

        def in_x_time(x):
            return (x_min <= x <= x_max)

        def calc_y_range_multi(pairs):
            ys = []
            for x_list, y_list in pairs:
                if not x_list:
                    continue
                ys.extend([y for x, y in zip(x_list, y_list) if in_x_time(x)])
            if not ys:
                return None
            y_min = min(ys)
            y_max = max(ys)
            if y_min == y_max:
                pad = max(1e-3, abs(y_min) * 0.05)
                y_min -= pad
                y_max += pad
            else:
                pad = 0.05 * (y_max - y_min)
                y_min -= pad
                y_max += pad
            return (y_min, y_max)

        any_scaled = False

        xs_ref = []
        if self.cb_ref_show.isChecked() and self.t_ref:
            xs_ref = [tr - t0 for tr in self.t_ref]

        if xs and self.cb_pos.isChecked():
            yr = calc_y_range_multi([(xs, list(self.pos_buf))])
            if yr:
                self.vb_pos.setYRange(yr[0], yr[1], padding=0)
                any_scaled = True

        spd_pairs = []
        if xs and self.cb_spd.isChecked():
            spd_pairs.append((xs, list(self.spd_buf)))
        if xs_ref and self.cb_ref_show.isChecked() and self.ref_view == self.vb_spd:
            spd_pairs.append((xs_ref, list(self.y_ref)))
        if spd_pairs:
            yr = calc_y_range_multi(spd_pairs)
            if yr:
                self.vb_spd.setYRange(yr[0], yr[1], padding=0)
                any_scaled = True

        cur_pairs = []
        if xs and self.cb_cur.isChecked():
            cur_pairs.append((xs, list(self.cur_buf)))
        if xs_ref and self.cb_ref_show.isChecked() and self.ref_view == self.vb_cur:
            cur_pairs.append((xs_ref, list(self.y_ref)))
        if cur_pairs:
            yr = calc_y_range_multi(cur_pairs)
            if yr:
                self.vb_cur.setYRange(yr[0], yr[1], padding=0)
                any_scaled = True

        if xs and self.cb_tmp.isChecked():
            yr = calc_y_range_multi([(xs, list(self.tmp_buf))])
            if yr:
                self.vb_tmp.setYRange(yr[0], yr[1], padding=0)
                any_scaled = True

        if xs_ref and self.cb_ref_show.isChecked() and self.ref_view == self.vb_ref:
            yr = calc_y_range_multi([(xs_ref, list(self.y_ref))])
            if yr:
                self.vb_ref.setYRange(yr[0], yr[1], padding=0)
                any_scaled = True

        if not any_scaled:
            self._log("Auto Y: 현재 X범위/표시곡선 내 데이터가 없습니다.")

    def _reset_ref_amp_for_mode(self, mode_id: int):
        if mode_id == MODE_CURRENT:
            self.spin_amp.setSingleStep(0.1)
            self.spin_amp.setValue(0.1)
        elif mode_id == MODE_VELOCITY:
            self.spin_amp.setSingleStep(100.0)
            self.spin_amp.setValue(500.0)

    def _set_ref_axis_target(self, mode_id: int):
        if mode_id == MODE_CURRENT:
            target = self.vb_cur
        elif mode_id == MODE_VELOCITY:
            target = self.vb_spd
        else:
            target = self.vb_ref
        if self.ref_view is not None and self.ref_view is not target:
            self.ref_view.removeItem(self.curve_ref)
        if self.ref_view is not target:
            target.addItem(self.curve_ref)
        self.ref_view = target
