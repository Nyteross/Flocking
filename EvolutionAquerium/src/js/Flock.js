/**
 * @class Flock
 * handles flocking behavior
 */
class Flock {

  constructor(currentAgent) {
    this.currentAgent = currentAgent;
    this.wandertheta = 0;
  }

  /**
   * @method seek()
   * @param {*} target 
   * simple method to seek something
   */
  seek(target) {
    let desired = null;
    desired = Vector.sub(target.pos, this.currentAgent.pos);
    desired.normalize();
    desired.mult(this.currentAgent.maxSpeed);
    let steer = Vector.sub(desired, this.currentAgent.vel);
    steer.limit(this.currentAgent.maxForce);
    return steer;
  }

  /**
   * just a basic refator
   * @param {*} sum 
   */
  _returnSteer(sum) {
    sum.normalize();
    sum.mult(this.currentAgent.maxSpeed);
    let steer = Vector.sub(sum, this.currentAgent.vel);
    steer.limit(this.currentAgent.maxForce);
    return steer;
  }

  /**
   * @method wander()
   * not in used
   */
  wander() {
    let wanderR = 25;
    let wanderD = 80;
    let change = 0.1;
    this.wandertheta += random(-change, change);

    // Now we have to calculate the new location to steer towards on the wander circle
    let circleloc = this.currentAgent.vel.copy();
    circleloc.normalize();
    circleloc.mult(wanderD);
    circleloc.add(this.currentAgent.pos);

    let h = this.currentAgent.vel.heading();

    let circleOffSet = new Vector(wanderR * Math.cos(this.wandertheta + h), wanderR * Math.sin(this.wandertheta + h));
    let target = Vector.add(circleloc, circleOffSet);

    // SEEK (have to make the seek function generic)
    let desired = null;
    desired = Vector.sub(target, this.currentAgent.pos);
    desired.normalize();
    desired.mult(this.currentAgent.maxSpeed);
    let steer = Vector.sub(desired, this.currentAgent.vel);
    steer.limit(this.currentAgent.maxForce);
    return steer;
  }

  /**
   * @method calculerSeparation()
   * @param {Array} agents 
   * part of flocking system
   */
  calculerSeparation(agents) {
    let desiredseperation = this.currentAgent.radius * 4;
    let sum = new Vector(); 
    let count = 0;
    for (let i = 0; i < agents.length; i++) {  // Parcourir tous les agents 
      let d = Vector.distSq(this.currentAgent.pos, agents[i].pos); // Distance au carré avec l'agent i
      if ((d > 0) && (d < desiredseperation * desiredseperation)) { // Si d est en dessous du carré de la separation voulu
        let diff = Vector.sub(this.currentAgent.pos, agents[i].pos); // Soustraction de la position de l'agent actuel à la position de l'agent i
        diff.normalize(); // Calcule de l'hypothénuse de la différence de position entre les deux agents et définition de la position x,y de diff à la position actuel divisé par l'hypoténuse
        diff.div(d); // division de la différence de position entre les deux agents par le carré de la distance entre les deux agents
        sum.add(diff); // On ajoute la position de la diff au vecteur sum
        count++;
      }
    }
    if (count > 0) {
      sum.div(count); // Moyenne de des vecteurs voisins 
      return this._returnSteer(sum);
    }
    return new Vector(0, 0);
  };

  /**
   * @method calculerAlignement()
   * @param {Array} agents 
   * part of flocking system
   */
  calculerAlignement(agents) {
    let neighbordist = 50;
    let sum = new Vector(0, 0);
    let count = 0;
    for (let i = 0; i < agents.length; i++) { // Parcourir tous les agents
      let d = Vector.distSq(this.currentAgent.pos, agents[i].pos); // Distance au carré avec l'agent i
      if ((d > 0) && (d < neighbordist * neighbordist)) { // Si d est en dessous du carré de la distance entre deux voisins
        sum.add(agents[i].vel); // On ajoute la vélocité d'un des agents au vecteur sum
        count++;
      }
    }
    if (count > 0) {
      sum.div(count); // Moyenne de des vecteurs voisins 
      return this._returnSteer(sum);
    }
    return new Vector(0, 0);
  }


  /**
   * @method cohesion()
   * @param {Array} agents 
   * part of flocking system
   */
  calculerCohesion(agents) {
    let neighbordist = 30; 
    let sum = new Vector(0, 0);
    let count = 0;
    for (let i = 0; i < agents.length; i++) { // Parcourir tous les agents
      let d = Vector.distSq(this.currentAgent.pos, agents[i].pos); // Distance au carré avec l'agent i
      if ((d > 0) && (d < neighbordist * neighbordist)) { // Si d est en dessous du carré de la distance entre deux voisins
        sum.add(agents[i].pos); // On ajoute la position d'un des agents au vecteur sum
        count++;
      }
    }
    if (count > 0) {
      sum.div(count); // Moyenne de des vecteurs voisins 
      sum.sub(this.currentAgent.pos); // on soustrait la position de l'agent actuel au vecteur sum
      return this._returnSteer(sum);
    }
    return new Vector(0, 0);
  }
}