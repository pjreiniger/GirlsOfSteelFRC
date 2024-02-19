/* eslint-disable no-param-reassign */
/* eslint-disable no-underscore-dangle */
import { LitElement } from 'lit';
import { property, state } from 'lit/decorators.js';
import getPoses from './get-poses';
import { FieldObjectApi } from '@frc-web-components/fwc/field-interfaces';

export class FieldImagePoses extends LitElement {
  @property({ type: Array }) poses: Uint8Array | number[] = [];
  @property({ type: String }) unit: string | null = 'inherit';
  @property({ type: String, attribute: 'rotation-unit' }) rotationUnit:
    | string
    | null = 'inherit';
  @property({ type: String }) image = '';
  @property({ type: Number }) width = 0.6;
  @property({ type: Number }) length = 0.9;
  @property({ type: Number }) opacity = 0.7;

  @state() _poses: (Uint8Array | number[])[] = [];
  @state() _image: Image = undefined;

  firstUpdated() {
    const image = new Image();
    image.src = this.image;
    console.log(image);
    this._image = image;

  }

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
    const rotationUnit =
      this.rotationUnit === 'inherit' || this.rotationUnit === null
        ? parentRotationUnit
        : this.rotationUnit;

    canvas.globalAlpha = Math.max(0, Math.min(1, this.opacity));
    canvas.fillStyle = '#222';
    canvas.strokeStyle = "purple";
    
    const imgWidth = lengthToPx(this.width, unit);
    const imgHeight = lengthToPx(this.length, unit);

    if (this._image) {
      for (let i = 0; i < this._poses.length; i += 1) {
        const pose = this._poses[i];
        const [x, y] = pose;
        const angle = rotationUnit === 'rad' ? pose[2] : pose[2] / (180 / Math.PI);
        canvas.save();
        
        canvas.translate(xToPx(x, unit), yToPx(y, unit));
        canvas.rotate(-angle);
        canvas.translate(-xToPx(x, unit), -yToPx(y, unit));
        canvas.drawImage(this._image, xToPx(x, unit) - imgWidth / 2, yToPx(y, unit) - imgHeight / 2, imgWidth, imgHeight)

        canvas.restore();
      }
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
