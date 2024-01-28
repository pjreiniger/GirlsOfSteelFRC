
export const elementConfig = {
  dashboard: {
    displayName: "SuperStructure",
  },
  properties: {
    pivotMotorAngle: { type: "Number", primary: true },
    goalAngle: { type: "Number", reflect: true },
    shooterMotorPercentage: { type: "Number", reflect: true },
    pivotMotorPercentage: { type: "Number", reflect: true },
    hasGamePiece: { type: "Boolean", reflect: true },
    intakeMotorPercentage: { type: "Number", reflect: true },
  },
  demos: [
    {
      html: `
      <super-structure source-key="/SuperStructure" source-provider="NetworkTables" style="width: 460px; height: 315px;">
      </super-structure>
    `,
    },
  ],
};