export class Matrix {
    constructor(matrix) {
        this.matrix = matrix;
    }
    mmul(matrix) {
        this.matrix = this.matrix.map((row, i) => matrix.asArray()[0].map((_, j) => row.reduce((acc, _, n) => acc + this.matrix[i][n] * matrix.asArray()[n][j], 0)));
        return this;
    }
    asArray() {
        return this.matrix;
    }
    get(rowIndex, columnIndex) {
        return this.asArray()[rowIndex][columnIndex];
    }
    static multiply(...matrices) {
        let m = matrices[0];
        for (let i = 1; i < matrices.length; i++) {
            m = m.mmul(matrices[i]);
        }
        return m;
    }
    static scaleMatrix(x, y) {
        return new Matrix([
            [x, 0, 0],
            [0, y, 0],
            [0, 0, 1]
        ]);
    }
    static translateMatrix(x, y) {
        return new Matrix([
            [1, 0, x],
            [0, 1, y],
            [0, 0, 1]
        ]);
    }
    static rotateMatrix(deg) {
        return new Matrix([
            [Math.cos(deg), -1 * Math.sin(deg), 0],
            [Math.sin(deg), Math.cos(deg), 0],
            [0, 0, 1]
        ]);
    }
    static createScaleMatrix(x, y, origin) {
        return this.multiply(Matrix.translateMatrix(origin.x, origin.y), Matrix.scaleMatrix(x, y), Matrix.translateMatrix(-origin.x, -origin.y));
    }
    static createRotateMatrix(deg, origin) {
        return this.multiply(Matrix.translateMatrix(origin.x, origin.y), Matrix.rotateMatrix(deg), Matrix.translateMatrix(-origin.x, -origin.y));
    }
}
//# sourceMappingURL=Matrix.js.map