package main

import (
    "fmt"
    "math"
)

type Quaternion struct {
    W, X, Y, Z float64
}

// Construction of unit quaternion from Euler angles,
//  sequence (1,2,3) / (R,P,Y) in radians
func fromRPY(r, p, y float64) Quaternion {
    sy2 := math.Sin(y / 2.0)
    sp2 := math.Sin(p / 2.0)
    sr2 := math.Sin(r / 2.0)

    cy2 := math.Cos(y / 2.0)
    cp2 := math.Cos(p / 2.0)
    cr2 := math.Cos(r / 2.0)

    qRPY := Quaternion {
        cy2 * cp2 * cr2 + sy2 * sp2 * sr2,
        cy2 * cp2 * sr2 - sy2 * sp2 * cr2,
        cy2 * sp2 * cr2 + sy2 * cp2 * sr2,
        sy2 * cp2 * cr2 - cy2 * sp2 * sr2,
    }
    return qRPY
}

// Construction of unit quaternion from Euler angles,
//  sequence (3,2,1) / (R,P,Y) in radians
func fromYPR(y, p, r float64) Quaternion {
    sy2 := math.Sin(y / 2.0)
    sp2 := math.Sin(p / 2.0)
    sr2 := math.Sin(r / 2.0)

    cy2 := math.Cos(y / 2.0)
    cp2 := math.Cos(p / 2.0)
    cr2 := math.Cos(r / 2.0)

    qYPR := Quaternion {
        cy2 * cp2 * cr2 - sy2 * sp2 * sr2,
        sy2 * cp2 * cr2 + cy2 * sp2 * sr2,
        cy2 * sp2 * cr2 - sy2 * cp2 * sr2,
        sy2 * sp2 * cr2 + cy2 * cp2 * sr2,
    }
    return qYPR
}

// Construction of a unit quaternion to represent
//  a rotation of ϴ radians about the unit vector n
func fromAxisAngle(nx, ny, nz, ϴ float64) Quaternion {
    cϴ2 := math.Cos(ϴ / 2.0)
    sϴ2 := math.Sin(ϴ / 2.0)

    qAϴ := Quaternion {
        cϴ2,
        nx * sϴ2,
        ny * sϴ2,
        nz * sϴ2,
    }
    return qAϴ
}

func (quat Quaternion) getAxisAngle() (nx, ny, nz, ϴ float64) {
    ϴ = 2.0 * math.Acos(quat.W)
    sqrtdiff := math.Sqrt(1.0 - quat.W * quat.W)
    nx = quat.X / sqrtdiff
    ny = quat.Y / sqrtdiff
    nz = quat.Z / sqrtdiff
    return
}

func getAxisAngle(quat Quaternion) (nx, ny, nz, ϴ float64) {
    nx, ny, nz, ϴ = quat.getAxisAngle()
    return
}

func (quat *Quaternion) Conjugate() {
    quat.Star()
}

func (quat *Quaternion) Star() {
    quat.X *= -1.0
    quat.Y *= -1.0
    quat.Z *= -1.0
}

func Star(quat Quaternion) Quaternion {
    q_n := quat
    q_n.Star()
    return q_n
}

func (quat *Quaternion) Plus(other Quaternion) {
    quat.W += other.W
    quat.X += other.X
    quat.Y += other.Y
    quat.Z += other.Z
}

func Sum(aa, bb Quaternion) Quaternion {
    sum_q := aa
    sum_q.Plus(bb)
    return sum_q
}

// Non-commutative quaternion product (qq * pp)
func (qq *Quaternion) Times(pp Quaternion) {
    qq.W = qq.W * pp.W - qq.X * pp.X - qq.Y * pp.Y - qq.Z * pp.Z
    qq.X = qq.X * pp.W - qq.W * pp.X - qq.Z * pp.Y - qq.Y * pp.Z
    qq.Y = qq.Y * pp.W - qq.Z * pp.X - qq.W * pp.Y - qq.X * pp.Z
    qq.Z = qq.Z * pp.W - qq.Y * pp.X - qq.X * pp.Y - qq.W * pp.Z
}

func Product(aa, bb Quaternion) Quaternion {
    prod_q := aa
    prod_q.Times(bb)
    return prod_q
}

func (qq Quaternion) Length() float64 {
    return Length(qq)
}

func Length(qq Quaternion) float64 {
    q_l := math.Sqrt( qq.W * qq.W + qq.X * qq.X + qq.Y * qq.Y + qq.Z * qq.Z )
    return q_l
}

// Divide all elements by a scalar
func (qq *Quaternion) Divide(dd float64) {
    qq.W /= dd
    qq.X /= dd
    qq.Y /= dd
    qq.Z /= dd
}

func (qq *Quaternion) Invert() {
    q_str := Star(*qq)
    q_len := qq.Length()
    q_lsq := q_len * q_len
    q_new := q_str
    q_new.Divide(q_lsq)
    *qq = q_new
}

func main() {
    q1 := Quaternion{1.0, 0.0, 0.0, 1.0}
    q1.X = 1.0

    fmt.Println(q1)

    q1.Star()
    fmt.Println(q1)

    q1.Conjugate()
    fmt.Println(q1)

    q2 := Quaternion{1.0, 0.0, 1.0, 0.0}
    q1.Plus(q2)

    fmt.Println(q1)
    fmt.Println(q2)

    q3 := Sum(q1, q2)
    fmt.Println(q3)

    q1.Times(q2)
    fmt.Println(q1)

    fmt.Println(q1.Length())

    q1a := q1
    q1a.Times(q2)

    fmt.Println(q1)
    fmt.Println(q1a)

    q1.Invert()
    fmt.Println(q1)
}