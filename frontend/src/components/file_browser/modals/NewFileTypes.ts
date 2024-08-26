export class CardEntryProps {
  public value: string;
  public id: string;
  public icon: any;
  public text: string;

  constructor(value: string, id: string, icon: any, text: string) {
    this.value = value;
    this.id = id;
    this.icon = icon;
    this.text = text;
  }
}
