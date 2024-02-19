/* eslint-disable no-param-reassign */
/* eslint-disable no-underscore-dangle */
import { LitElement } from 'lit';
import { property, state } from 'lit/decorators.js';
import getPoses from './get-poses';
import { FieldObjectApi } from '@frc-web-components/fwc/field-interfaces';

export class FieldImagePoses extends LitElement {
  @property({ type: Array }) poses: Uint8Array | number[] = [];
  @property({ type: String }) unit: string | null = 'inherit';
  @property({ type: String }) image = '';
  @property({ type: Number }) width = 0.6;
  @property({ type: Number }) length = 0.9;
  @property({ type: Number }) opacity = 0.7;

  @state() _poses: (Uint8Array | number[])[] = [];

  updated(changedProps: Map<string, unknown>) {
    console.log("Updating... trying to get pose????" + changedProps.has('poses'))
    if (changedProps.has('poses')) {
      this._poses = getPoses(this.poses);
    }
    console.log(this._poses)
    console.log("Done")
  }

  draw({
    canvas,
    unit: parentUnit,
    rotationUnit: parentRotationUnit,
    xToPx,
    yToPx,
    lengthToPx,
    origin,
  }: FieldObjectApi): void {
    const unit =
      this.unit === 'inherit' || this.unit === null ? parentUnit : this.unit;

//     canvas.lineWidth = this.lineWidth;
//     canvas.strokeStyle = this.color;
//     canvas.globalAlpha = this.opacity;

    canvas.globalAlpha = Math.max(0, Math.min(1, this.opacity));
    canvas.fillStyle = '#222';
    canvas.strokeStyle = "purple";
    canvas.lineWidth = lengthToPx(3, 'in');

    console.log("Drawing3...");

    if (this._poses.length > 1) {
      for (let i = 0; i < this._poses.length - 1; i += 1) {
        const [x1, y1] = this._poses[i];
        const [x2, y2] = this._poses[i + 1];
        canvas.moveTo(xToPx(x1, unit), yToPx(y1, unit));
        canvas.lineTo(xToPx(x2, unit), yToPx(y2, unit));
      }

      canvas.stroke();
    }
  }
}

if (!customElements.get('gos-field-image-poses')) {
  customElements.define('gos-field-image-poses', FieldImagePoses);
}

declare global {
  interface HTMLElementTagNameMap {
    'gos-field-image-poses': FieldImagePoses;
  }
}
